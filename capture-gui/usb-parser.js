// USB 1.x low/full-speed decoder for packed D+/D- samples.
// Sample bytes are four 2-bit states in chronological order, least-significant pair first:
// bit 0 = D+, bit 1 = D-. FS J=01/K=10; LS J=10/K=01; SE0=00; SE1=11.
// A 48 MHz capture contains 4 samples per FS bit and 32 per LS bit.
const PID_NAMES = {1:'OUT',9:'IN',5:'SOF',13:'SETUP',3:'DATA0',11:'DATA1',7:'DATA2',15:'MDATA',2:'ACK',10:'NAK',14:'STALL',6:'NYET',12:'PRE',0:'ERR',4:'SPLIT',8:'PING'};
const TOKEN_PIDS = new Set([1,4,5,9,13]);
const DATA_PIDS = new Set([3,7,11,15]);
const HANDSHAKE_PIDS = new Set([2,6,10,14]);

export function unpackSamples(bytes) {
  const samples = new Uint8Array(bytes.length * 4);
  let n = 0;
  for (const b of bytes) for (let q=0;q<4;q++) samples[n++] = (b >>> (q*2)) & 3;
  return samples;
}

function crc5(bits) {
  let crc = 0x1f;
  for (const bit of bits) { const mix=(crc&1)^bit; crc>>=1; if(mix) crc^=0x14; }
  return (crc ^ 0x1f) & 0x1f;
}
function crc16(bytes) {
  let crc=0xffff;
  for(const byte of bytes) for(let i=0;i<8;i++){const mix=(crc&1)^((byte>>i)&1);crc>>=1;if(mix)crc^=0xa001;}
  return (crc^0xffff)&0xffff;
}
function bitsToBytes(bits) { const out=[]; for(let i=0;i+7<bits.length;i+=8){let b=0;for(let j=0;j<8;j++)b|=bits[i+j]<<j;out.push(b);} return out; }
function sampleAt(samples, idx) { return idx>=0&&idx<samples.length?samples[idx]:3; }
function packetDecode(samples, edge, speed, nominalSamplesPerBit, sampleRateHz, index) {
  // Recover bit timing from NRZI transitions. Every transition marks a zero
  // bit; the USB SYNC field provides seven consecutive one-bit intervals to
  // acquire the sender's clock before payload decoding begins.
  let idleIndex=edge-1, leadingSe1=0;
  while(idleIndex>=0&&samples[idleIndex]===3){idleIndex--;leadingSe1++;}
  if(leadingSe1>=Math.max(2,Math.ceil(nominalSamplesPerBit*0.6)))return null;
  const idleState=sampleAt(samples,idleIndex), firstState=sampleAt(samples,edge);
  if((idleState!==1&&idleState!==2)||(firstState!==1&&firstState!==2))return null;
  const rawBits=[0];
  const errors=[];
  let period=nominalSamplesPerBit,lastEdge=edge,lastState=firstState,eopStart=null,eopJStartSample=null,endSample=null;
  let clockSamples=0,clockBits=0;
  const minStableSamples=Math.max(2,Math.ceil(nominalSamplesPerBit*0.6));
  const sync=[0,0,0,0,0,0,0,1];
  const addTransition=(nextEdge,nextState)=>{
    const delta=nextEdge-lastEdge;
    let bitCount;
    if(rawBits.length<7){
      bitCount=1;
      clockSamples+=delta;
      clockBits++;
      period=clockSamples/clockBits;
    }else{
      bitCount=Math.round(delta/period);
      if(bitCount<1||bitCount>64)return false;
      // Track the sender's average bit period from transition-to-transition
      // measurements. A cumulative estimate tolerates the ±1 sample
      // quantization at 4x oversampling and follows small clock offsets.
      clockSamples+=delta;
      clockBits+=bitCount;
      const measured=clockSamples/clockBits;
      period=Math.max(nominalSamplesPerBit*0.8,Math.min(nominalSamplesPerBit*1.2,measured));
    }
    for(let i=1;i<bitCount;i++)rawBits.push(1);
    rawBits.push(0);
    for(let i=0;i<Math.min(rawBits.length,sync.length);i++)if(rawBits[i]!==sync[i])return false;
    lastEdge=nextEdge;lastState=nextState;
    return true;
  };
  for(let cursor=edge+1;cursor<samples.length;cursor++){
    const state=samples[cursor];
    if(state===3){
      let se1End=cursor+1;
      while(se1End<samples.length&&samples[se1End]===3)se1End++;
      if(se1End-cursor>=minStableSamples){errors.push('SE1 line state');return null;}
      // D+ and D− are sampled as separate GPIO bits. A one-sample SE1 can be
      // caused by their input paths changing on adjacent capture clocks;
      // ignore that skew and use the next stable J/K level as the edge.
      cursor=se1End-1;
      continue;
    }
    if(state===0){
      let se0End=cursor+1;
      while(se0End<samples.length&&samples[se0End]===0)se0End++;
      const se0Length=se0End-cursor;
      // Ignore brief SE0 glitches; a real USB EOP holds SE0 for about two
      // bit times. Keep scanning from the state after the glitch.
      if(se0Length<minStableSamples&&se0End<samples.length&&(samples[se0End]===1||samples[se0End]===2)){
        cursor=se0End-1;
        continue;
      }
      eopStart=cursor;
      const delta=eopStart-lastEdge, bitCount=Math.round(delta/period);
      if(bitCount<1||bitCount>64)return null;
      for(let i=1;i<bitCount;i++)rawBits.push(1);
      let jStart=cursor;
      while(jStart<samples.length&&samples[jStart]===0)jStart++;
      const eopSamples=jStart-eopStart;
      if(eopSamples<period*1.5)errors.push('Short EOP');
      if(jStart>=samples.length||samples[jStart]!==idleState)errors.push('Missing EOP idle J');
      else eopJStartSample=jStart;
      endSample=eopJStartSample===null?jStart:Math.round(eopJStartSample+period);
      break;
    }
    if(state===lastState)continue;
    let runEnd=cursor+1;
    while(runEnd<samples.length&&samples[runEnd]===state)runEnd++;
    const runLength=runEnd-cursor;
    if(runLength<minStableSamples){cursor=runEnd-1;continue;}
    if(!addTransition(cursor,state))return null;
    if(rawBits.length>1408)return null;

    // Pico-PIO-USB transmits PRE without an EOP. Recognize it as soon as the
    // complete SYNC and PID have arrived, before later LS traffic is scanned.
    if(rawBits.length>=16&&speed==='FS'&&bitsToByte(rawBits.slice(8,16))===0x3c){
      if(period<nominalSamplesPerBit*0.8||period>nominalSamplesPerBit*1.2)return null;
      endSample=Math.round(edge+16*period);
      return {index,pid:12,name:'PRE',type:'Special',addr:null,endpoint:null,payload:[],pidByte:0x3c,bytes:[0x3c],errors:[],startSample:edge,endSample,eopJStartSample:null,lengthSamples:endSample-edge,startTimeUs:edge/sampleRateHz*1e6,startBitTime:edge/nominalSamplesPerBit,lengthBitTime:(endSample-edge)/period,phase:0,speed,samplesPerBit:period,sampleRateHz,sync:'80'};
    }
    if(rawBits.length>=8&&rawBits.slice(0,8).some((bit,i)=>bit!==sync[i]))return null;
  }
  if(rawBits.length<16)return null;
  if(period<nominalSamplesPerBit*0.8||period>nominalSamplesPerBit*1.2)return null;
  if(eopStart===null){errors.push('EOP not found');endSample=samples.length;}
  const bits=[],postSync=rawBits.slice(8);let ones=0;
  for(const bit of postSync){
    if(ones===6){if(bit!==0)errors.push('Bit stuffing error: expected stuffed zero');else{ones=0;continue;}}
    bits.push(bit);ones=bit?ones+1:0;
  }
  const bytes=bitsToBytes(bits), pidByte=bytes[0]??null, pid=pidByte===null?null:pidByte&15;
  if(pidByte!==null&&(((pidByte>>4)^pid)!==15)) errors.push('PID complement mismatch');
  const name=pid===null?'Unknown':(PID_NAMES[pid]??`PID 0x${pid.toString(16)}`);
  let addr=null, endpoint=null, payload=[], expectedCrc=null, actualCrc=null;
  if(pid!==null&&TOKEN_PIDS.has(pid)){
    if(bits.length!==24)errors.push('Token length mismatch');
    if(bits.length>=24){
      const token=bits.slice(8,19), c=bits.slice(19,24); addr=bitsToInt(token.slice(0,7));endpoint=bitsToInt(token.slice(7,11));expectedCrc=bitsToInt(c);actualCrc=crc5(token);if(expectedCrc!==actualCrc)errors.push('CRC5 mismatch');
    }
  } else if(pid!==null&&DATA_PIDS.has(pid)){
    if(bits.length<24)errors.push('Data packet too short');
    if((bits.length-8)%8!==0)errors.push('Data packet bit length is not byte aligned');
    const body=bytes.slice(1);if(body.length>=2){payload=body.slice(0,-2);actualCrc=body[body.length-2]|(body[body.length-1]<<8);expectedCrc=crc16(payload);if(actualCrc!==expectedCrc)errors.push('CRC16 mismatch');}
  } else if(pid!==null&&HANDSHAKE_PIDS.has(pid)&&bits.length!==8) errors.push('Unexpected handshake payload');
  return {index,pid,name,type:pid!==null&&TOKEN_PIDS.has(pid)?'Token':pid!==null&&DATA_PIDS.has(pid)?'Data':pid!==null&&HANDSHAKE_PIDS.has(pid)?'Handshake':'Special',addr,endpoint,payload,pidByte,bytes,errors,startSample:edge,endSample,eopJStartSample,lengthSamples:endSample-edge,startTimeUs:edge/sampleRateHz*1e6,startBitTime:edge/nominalSamplesPerBit,lengthBitTime:(endSample-edge)/period,phase:0,speed,samplesPerBit:period,sampleRateHz,sync:'80'};
}
function bitsToByte(bits){let b=0;for(let i=0;i<bits.length;i++)b|=bits[i]<<i;return b;}
function bitsToInt(bits){let b=0;for(let i=0;i<bits.length;i++)b|=bits[i]<<i;return b;}

function gapFromEop(previous,next){
  if(previous?.eopJStartSample==null||!next)return null;
  const sampleCount=next.startSample-previous.eopJStartSample;
  return {sampleCount,bitTimes:sampleCount/next.samplesPerBit,timeUs:sampleCount/next.sampleRateHz*1e6};
}
function packetScore(packet){
  let score=packet.errors.length*10;
  if(packet.errors.includes('EOP not found'))score+=500;
  if(packet.errors.includes('PID complement mismatch'))score+=200;
  if(packet.errors.some(e=>e.startsWith('CRC')))score+=100;
  if(packet.errors.some(e=>e.includes('length mismatch')||e.includes('not byte aligned')||e.includes('handshake payload')))score+=100;
  if(packet.errors.some(e=>e.startsWith('Bit stuffing')))score+=100;
  if(packet.pid===null||packet.name.startsWith('PID 0x'))score+=100;
  return score;
}

export function decodeCapture(packedBytes, {speed='FS',sampleRateHz}={}) {
  const samples=unpackSamples(packedBytes), packets=[];
  const actualSampleRate=sampleRateHz??(speed==='LS'?6000000:48000000);
  const modes=[['FS',12_000_000],['LS',1_500_000]]
    .map(([mode,bitRate])=>({speed:mode,samplesPerBit:actualSampleRate/bitRate}))
    .filter(mode=>Number.isInteger(mode.samplesPerBit)&&mode.samplesPerBit>=4);
  if(!modes.length)modes.push({speed,samplesPerBit:4});
  let lastEnd=-1, previousPacket=null;
  for(let edge=1;edge<samples.length-8;edge++){
    if(edge<=lastEnd)continue;
    let found=null,foundScore=Infinity;
    for(const mode of modes){
      if(samples[edge-1]===samples[edge]||samples[edge-1]===0||samples[edge]===0||samples[edge]===3)continue;
      // A malformed edge candidate must never abort decoding the rest of a
      // frozen capture (or stop the continuous-capture loop in the UI).
      let candidate;
      try{candidate=packetDecode(samples,edge,mode.speed,mode.samplesPerBit,actualSampleRate,packets.length+1);}
      catch{continue;}
      if(!candidate)continue;
      const score=packetScore(candidate);
      if(score<foundScore){found=candidate;foundScore=score;}
    }
    if(found){
      if(found.startSample-lastEnd<found.samplesPerBit&&packets.length)continue;
      const gap=gapFromEop(previousPacket,found);
      found.gapBitTime=gap?.bitTimes??null;
      found.gapTimeUs=gap?.timeUs??null;
      previousPacket=found;
      lastEnd=found.endSample;
      // SOF is present in the continuous physical sample stream for timing,
      // but is excluded from the retained packet list by capture policy.
      if(found.name==='SOF')continue;
      packets.push(found);
    }
  }
  annotateTransactions(packets);
  return packets;
}

function annotateTransactions(packets) {
  let active=null, pendingPreambles=[], transactionId=0;
  const finish=(result)=>{
    if(!active)return;
    const names=active.packets.map(p=>p.name).join(' → ');
    const tokenName=active.packets.find(p=>p.type==='Token')?.name??active.packets[0].name;
    const data=active.packets.find(p=>p.type==='Data');
    const handshake=active.packets.find(p=>p.type==='Handshake');
    const dataAckGap=result==='ACK'?gapFromEop(data,handshake):null;
    for(const packet of active.packets){
      packet.transactionId=transactionId;
      packet.transactionResult=result;
      packet.transactionToken=tokenName;
      packet.transactionLabel=names;
      packet.dataAckGapBitTime=dataAckGap?.bitTimes??null;
      packet.dataAckGapTimeUs=dataAckGap?.timeUs??null;
    }
    transactionId++;
    active=null;
  };
  for(const packet of packets){
    if(packet.name==='PRE'){
      if(active)active.packets.push(packet);
      else pendingPreambles.push(packet);
      continue;
    }
    if(packet.type==='Token'){
      finish('Incomplete');
      const preambles=packet.speed==='LS'?pendingPreambles:[];
      active={packets:[...preambles,packet]};
      pendingPreambles=[];
      continue;
    }
    if(!active)continue;
    active.packets.push(packet);
    if(packet.type==='Handshake')finish(packet.name);
  }
  finish('Incomplete');
}

export function packetDetails(packet,{speed='FS'}={}) {
  if(!packet)return '';
  const hex=packet.bytes.map(b=>b.toString(16).padStart(2,'0')).join(' ');
  const packetSpeed=packet.speed??speed;
  return [`Packet ${packet.index} — ${packet.type} ${packet.name}`,`Speed: ${packetSpeed} (${packet.samplesPerBit??4} samples/bit)`,`Transaction: ${packet.transactionLabel??'unassociated'} (${packet.transactionResult??'unknown'})`,`Start: sample ${packet.startSample} (${fmt(packet.startTimeUs)} µs / ${fmt(packet.startBitTime)} ${packetSpeed} bit times)`,`Length: ${fmt(packet.lengthBitTime)} ${packetSpeed} bit times (${packet.lengthSamples} samples)`,`Gap from previous packet: ${formatGap(packet.gapBitTime,packetSpeed,packet.gapTimeUs)}`,`DATA→ACK gap: ${formatGap(packet.dataAckGapBitTime,packetSpeed,packet.dataAckGapTimeUs)}`,`PID: ${packet.pidByte===null?'missing':`0x${packet.pidByte.toString(16).padStart(2,'0')} (${packet.name})`}`,`Address: ${packet.addr??'—'}  Endpoint: ${packet.endpoint??'—'}`,`Payload: ${packet.payload.length?packet.payload.map(b=>b.toString(16).padStart(2,'0')).join(' '):'—'}`,`Bytes after SYNC: ${hex||'—'}`,`Diagnostics: ${packet.errors.length?packet.errors.join('; '):'none'}`].join('\n');
}
export function formatGap(bitTimes,speed='FS',timeUs) {
  if(bitTimes===null||bitTimes===undefined)return '—';
  const bitTimeUs=speed==='LS'?(1/1.5):(1/12);
  return `${fmt(bitTimes)} bt / ${(timeUs??bitTimes*bitTimeUs).toFixed(3)} µs`;
}
function fmt(n){return Number(n).toFixed(2);}

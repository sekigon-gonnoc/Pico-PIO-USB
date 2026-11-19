import test from 'node:test';
import assert from 'node:assert/strict';
import {decodeCapture,formatGap,packetDetails,unpackSamples} from '../usb-parser.js';
import {summarizeGaps,validInAckGaps} from '../ack-stats.js';

function pack(samples){const out=new Uint8Array(Math.ceil(samples.length/4));for(let i=0;i<samples.length;i++)out[i>>2]|=(samples[i]&3)<<((i&3)*2);return out;}
function appendBits(states,bits,speed='FS',samplesPerBit=4){
  const j=speed==='FS'?1:2,k=speed==='FS'?2:1;let prev=j;
  for(const bit of bits){if(bit===0)prev=prev===j?k:j;for(let i=0;i<samplesPerBit;i++)states.push(prev);}
  return prev;
}
function wirePacket(pid,body=[],speed='FS',{stuff=true,samplesPerBit=4,eop=true}={}){
  const bytes=[((~pid&15)<<4)|pid,...body],bits=[];for(const b of bytes)for(let i=0;i<8;i++)bits.push((b>>i)&1);
  return wirePacketBits(bits,speed,stuff,samplesPerBit,eop);
}
function wirePacketBits(bits,speed='FS',stuff=true,samplesPerBit=4,eop=true){
  const stuffed=[];let ones=0;for(const bit of bits){stuffed.push(bit);if(bit){ones++;if(ones===6){if(stuff)stuffed.push(0);ones=0;}}else ones=0;}
  const states=[];appendBits(states,[0,0,0,0,0,0,0,1,...stuffed],speed,samplesPerBit);if(eop){const j=speed==='FS'?1:2;states.push(...Array(samplesPerBit*2).fill(0),...Array(samplesPerBit).fill(j));}return states;
}
function wirePacketBitsWithDrift(bits,speed='FS',period=4.08){
  const j=speed==='FS'?1:2,k=speed==='FS'?2:1;let state=j,elapsed=0;
  const samples=[];
  const stuffed=[];let ones=0;
  for(const bit of bits){stuffed.push(bit);if(bit){if(++ones===6){stuffed.push(0);ones=0;}}else ones=0;}
  for(const bit of [0,0,0,0,0,0,0,1,...stuffed]){
    if(bit===0)state=state===j?k:j;
    const nextElapsed=elapsed+period,width=Math.round(nextElapsed)-Math.round(elapsed);
    samples.push(...Array(width).fill(state));elapsed=nextElapsed;
  }
  samples.push(...Array(Math.round(period*2)).fill(0),...Array(Math.round(period)).fill(j));
  return samples;
}
function bitsOf(n,width){return Array.from({length:width},(_,i)=>(n>>i)&1);}
function crc5(bits){let crc=0x1f;for(const bit of bits){const mix=(crc&1)^bit;crc>>=1;if(mix)crc^=0x14;}return(crc^0x1f)&0x1f;}
function crc16(bytes){let crc=0xffff;for(const byte of bytes)for(let i=0;i<8;i++){const mix=(crc&1)^((byte>>i)&1);crc>>=1;if(mix)crc^=0xa001;}return(crc^0xffff)&0xffff;}
test('unpacks 4 chronological 2-bit samples from each byte',()=>assert.deepEqual([...unpackSamples(new Uint8Array([0xe4]))],[0,1,2,3]));
test('decodes a minimal ACK packet and detects its EOP',()=>{
  const samples=[1,1,1,1,1,1,1,1,...wirePacket(2)];
  const packets=decodeCapture(pack(samples),{speed:'FS'});
  assert.equal(packets.length,1);assert.equal(packets[0].name,'ACK');assert.equal(packets[0].type,'Handshake');assert.deepEqual(packets[0].errors,[]);
  assert.equal(packets[0].lengthSamples,wirePacket(2).length);
});
test('decodes a low-speed handshake with opposite J/K polarity',()=>{
  const samples=[2,2,2,2,2,2,2,2,...wirePacket(10,[],'LS')];
  const packets=decodeCapture(pack(samples),{speed:'LS'});
  assert.equal(packets.length,1);assert.equal(packets[0].name,'NAK');
  assert.equal(packets[0].eopJStartSample,samples.length-4);
});
test('decodes LS packets at 32 samples per bit after an FS PRE in the same capture',()=>{
  const fields=[...bitsOf(3,7),...bitsOf(1,4)],token=[...fields,...bitsOf(crc5(fields),5)];
  const payload=[0x42],dataCrc=crc16(payload),data=[0xc3,...payload,dataCrc&255,dataCrc>>8];
  const lsStride=32;
  const pre=wirePacket(12,[],'FS',{samplesPerBit:4,eop:false});
  const inToken=wirePacketBits([...bitsOf(0x69,8),...token],'LS',true,lsStride);
  const dataPacket=wirePacket(3,data.slice(1),'LS',{samplesPerBit:lsStride});
  const ack=wirePacket(2,[],'LS',{samplesPerBit:lsStride});
  const samples=[...Array(8).fill(1),...pre,...Array(32).fill(1),...Array(96).fill(2),...inToken,...Array(96).fill(2),...dataPacket,...Array(96).fill(2),...ack];
  const packets=decodeCapture(pack(samples),{sampleRateHz:48_000_000});
  assert.deepEqual(packets.map(p=>p.name),['PRE','IN','DATA0','ACK']);
  assert.equal(packets[0].speed,'FS');assert.equal(packets[0].lengthBitTime,16);assert.deepEqual(packets[0].errors,[]);
  assert.ok(packets.slice(1).every(p=>p.speed==='LS'&&p.samplesPerBit===32&&p.errors.length===0));
  assert.ok(packets.every(p=>p.transactionToken==='IN'&&p.transactionLabel==='PRE → IN → DATA0 → ACK'));
  assert.ok(packets.every((p,i)=>i===0||p.startTimeUs>=packets[i-1].startTimeUs));
  assert.ok(packets[2].dataAckGapBitTime>0);
  assert.ok(packets[2].dataAckGapTimeUs>0);
});
test('decodes LS packets when captured D+ and D- polarity is reversed',()=>{
  const lsStride=32;
  const invert=states=>states.map(state=>state===1?2:state===2?1:state);
  const samples=[...Array(8).fill(1),...invert(wirePacket(10,[],'LS',{samplesPerBit:lsStride}))];
  const packets=decodeCapture(pack(samples),{sampleRateHz:48_000_000});
  assert.equal(packets.length,1);assert.equal(packets[0].name,'NAK');
  assert.equal(packets[0].speed,'LS');assert.equal(packets[0].samplesPerBit,32);
  assert.deepEqual(packets[0].errors,[]);
});
test('decodes token address, endpoint, and a valid CRC5',()=>{
  const fields=[...bitsOf(5,7),...bitsOf(2,4)],token=[...fields,...bitsOf(crc5(fields),5)];
  const samples=[1,1,1,1,1,1,1,1,...wirePacketBits([...bitsOf(0xE1,8),...token])];
  const [packet]=decodeCapture(pack(samples),{speed:'FS'});
  assert.equal(packet.name,'OUT');assert.equal(packet.addr,5);assert.equal(packet.endpoint,2);assert.deepEqual(packet.errors,[]);
});
test('excludes SOF packets from the decoded packet list',()=>{
  const fields=bitsOf(0x245,11),token=[...fields,...bitsOf(crc5(fields),5)];
  const samples=[1,1,1,1,1,1,1,1,...wirePacketBits([...bitsOf(0xa5,8),...token])];
  assert.deepEqual(decodeCapture(pack(samples),{speed:'FS'}),[]);
});
test('decodes data payload and verifies CRC16',()=>{
  const payload=[0x55,0xaa],crc=crc16(payload),bytes=[0xc3,...payload,crc&255,crc>>8];
  const samples=[1,1,1,1,1,1,1,1,...wirePacket(3,bytes.slice(1))];
  const [packet]=decodeCapture(pack(samples),{speed:'FS'});
  assert.equal(packet.name,'DATA0');assert.deepEqual(packet.payload,payload);assert.deepEqual(packet.errors,[]);
});
test('recovers a slightly drifting USB bit clock from NRZI transitions',()=>{
  const payload=[0x55,0xaa,0x7e],crc=crc16(payload);
  const bytes=[0xc3,...payload,crc&255,crc>>8],bits=[];
  for(const byte of bytes)for(let i=0;i<8;i++)bits.push((byte>>i)&1);
  const samples=[...Array(12).fill(1),...wirePacketBitsWithDrift(bits,'FS',4.08)];
  const [packet]=decodeCapture(pack(samples),{sampleRateHz:48_000_000});
  assert.equal(packet?.name,'DATA0');
  assert.deepEqual(packet?.payload,payload);
  assert.deepEqual(packet?.errors,[]);
  assert.ok(packet.samplesPerBit>4&&packet.samplesPerBit<4.2);
});
test('ignores one-sample J/K glitches while recovering NRZI transitions',()=>{
  const payload=[0x7e,0x55],crc=crc16(payload);
  const samples=[...Array(12).fill(1),...wirePacket(3,[...payload,crc&255,crc>>8])];
  // Add short opposite-polarity spikes. They are shorter than a USB bit and
  // should not be mistaken for recovered NRZI transitions.
  for(const at of [23,47,91,139,183,227]){
    if(samples[at]===1)samples[at]=2;else if(samples[at]===2)samples[at]=1;
  }
  const [packet]=decodeCapture(pack(samples),{sampleRateHz:48_000_000});
  assert.equal(packet?.name,'DATA0');
  assert.deepEqual(packet?.payload,payload);
  assert.deepEqual(packet?.errors,[]);
});
test('ignores one-sample SE1 caused by skew between D+ and D− input samples',()=>{
  const payload=[0x01,0x01,0x00],crc=crc16(payload);
  const samples=[...Array(12).fill(1),...wirePacket(3,[...payload,crc&255,crc>>8])];
  let edgeNumber=0;
  for(let i=1;i<samples.length-1;i++){
    if((samples[i-1]===1||samples[i-1]===2)&&(samples[i]===1||samples[i]===2)&&samples[i]!==samples[i-1]){
      if(edgeNumber++>6&&edgeNumber%3===0)samples[i]=3;
    }
  }
  const [packet]=decodeCapture(pack(samples),{sampleRateHz:48_000_000});
  assert.equal(packet?.name,'DATA0');
  assert.deepEqual(packet?.payload,payload);
  assert.deepEqual(packet?.errors,[]);
});
test('starts SYNC when the first sampled transition is a one-sample SE1',()=>{
  const payload=[0x01,0x01,0x00],crc=crc16(payload);
  const samples=[...Array(8).fill(1),...wirePacket(11,[...payload,crc&255,crc>>8])];
  // The first SYNC edge changes D+ and D− one capture clock apart.
  samples[8]=3;
  const [packet]=decodeCapture(pack(samples),{sampleRateHz:48_000_000});
  assert.equal(packet?.name,'DATA1');
  assert.deepEqual(packet?.payload,payload);
  assert.deepEqual(packet?.errors,[]);
});
test('reports CRC and bit-stuffing errors',()=>{
  const badCrc=[0x00,0x00];
  const crcSamples=[1,1,1,1,1,1,1,1,...wirePacket(3,[0x42,...badCrc])];
  const [crcPacket]=decodeCapture(pack(crcSamples),{speed:'FS'});
  assert.ok(crcPacket.errors.some(e=>e.includes('CRC16')));
  const unstufSamples=[1,1,1,1,1,1,1,1,...wirePacket(3,[0xff,0xff,0xff], 'FS',{stuff:false})];
  const [stuffPacket]=decodeCapture(pack(unstufSamples),{speed:'FS'});
  assert.ok(stuffPacket.errors.some(e=>e.includes('Bit stuffing')));
});
test('rejects packet-boundary candidates with trailing non-packet bits',()=>{
  const tokenFields=[...bitsOf(3,7),...bitsOf(1,4)],token=[...tokenFields,...bitsOf(crc5(tokenFields),5)];
  const tokenSamples=[...Array(8).fill(1),...wirePacketBits([...bitsOf(0x69,8),...token,0])];
  const [tokenPacket]=decodeCapture(pack(tokenSamples),{speed:'FS'});
  assert.equal(tokenPacket.name,'IN');
  assert.ok(tokenPacket.errors.includes('Token length mismatch'));

  const handshakeSamples=[...Array(8).fill(1),...wirePacketBits([...bitsOf(0xd2,8),1])];
  const [handshakePacket]=decodeCapture(pack(handshakeSamples),{speed:'FS'});
  assert.equal(handshakePacket.name,'ACK');
  assert.ok(handshakePacket.errors.includes('Unexpected handshake payload'));
});
test('groups an ACKed IN transaction and reports the DATA-to-ACK interval',()=>{
  const fields=[...bitsOf(3,7),...bitsOf(1,4)],token=[...fields,...bitsOf(crc5(fields),5)];
  const payload=[0x42],dataCrc=crc16(payload),data=[0xc3,...payload,dataCrc&255,dataCrc>>8];
  const parts=[wirePacketBits([...bitsOf(0x69,8),...token]),wirePacket(3,data.slice(1)),wirePacket(2)];
  const samples=[...Array(8).fill(1)];for(const part of parts)samples.push(...part,...Array(12).fill(1));
  const packets=decodeCapture(pack(samples),{speed:'FS'});
  assert.deepEqual(packets.map(p=>p.name),['IN','DATA0','ACK']);
  assert.ok(packets.every(p=>p.transactionResult==='ACK'));
  assert.deepEqual(packets.map(p=>p.transactionLabel),Array(3).fill('IN → DATA0 → ACK'));
  assert.ok(packets[2].gapBitTime>0);
  assert.equal(packets[1].dataAckGapBitTime,packets[2].gapBitTime);
  assert.match(packetDetails(packets[1],{speed:'FS'}),/DATA→ACK gap: [\d.]+ bt \/ [\d.]+ µs/);
  assert.deepEqual(validInAckGaps(packets),[4]);
  assert.deepEqual(validInAckGaps(packets.slice(1)),[]);

  // A one-sample-longer SE0 changes the real edge without moving ACK or the
  // decoder's nominal bit grid.
  const lateEopData=parts[1].slice();
  lateEopData[lateEopData.length-4]=0;
  const lateEopSamples=Array(8).fill(1);
  for(const part of [parts[0],lateEopData,parts[2]])lateEopSamples.push(...part,...Array(12).fill(1));
  const lateEopPackets=decodeCapture(pack(lateEopSamples));
  assert.equal(lateEopPackets[1].eopJStartSample,packets[1].eopJStartSample+1);
  assert.equal(lateEopPackets[2].startSample,packets[2].startSample);
  assert.deepEqual(validInAckGaps(lateEopPackets),[3.75]);

  // A J level without a direct SE0→J transition has no gap measurement.
  lateEopData[lateEopData.length-4]=3;
  const malformedSamples=Array(8).fill(1);
  for(const part of [parts[0],lateEopData,parts[2]])malformedSamples.push(...part,...Array(12).fill(1));
  assert.deepEqual(validInAckGaps(decodeCapture(pack(malformedSamples))),[]);

  const badParts=[parts[0],wirePacket(3,[0x42,0,0]),parts[2]];
  const badSamples=Array(8).fill(1);
  for(const part of badParts)badSamples.push(...part,...Array(12).fill(1));
  assert.deepEqual(validInAckGaps(decodeCapture(pack(badSamples))),[]);
});
test('marks NAKed IN transactions separately from ACK-completed transactions',()=>{
  const fields=[...bitsOf(3,7),...bitsOf(1,4)],token=[...fields,...bitsOf(crc5(fields),5)];
  const samples=[...Array(8).fill(1),...wirePacketBits([...bitsOf(0x69,8),...token]),...Array(12).fill(1),...wirePacket(10)];
  const packets=decodeCapture(pack(samples),{speed:'FS'});
  assert.deepEqual(packets.map(p=>p.name),['IN','NAK']);
  assert.ok(packets.every(p=>p.transactionResult==='NAK'));
});
test('formats gaps in bit times and microseconds for FS and LS',()=>{
  assert.equal(formatGap(1,'FS'),'1.00 bt / 0.083 µs');
  assert.equal(formatGap(1,'LS'),'1.00 bt / 0.667 µs');
  assert.equal(formatGap(null,'FS'),'—');
});
test('summarizes repeated measurements with weighted percentiles and standard deviation',()=>{
  assert.equal(summarizeGaps(new Map()),null);
  const stats=summarizeGaps(new Map([[12,1],[10,2],[11,1]]));
  assert.deepEqual(stats,{
    count:4,min:10,mean:10.75,median:10.5,p95:11.85,max:12,
    standardDeviation:Math.sqrt(0.6875),
  });
});

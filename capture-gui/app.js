import {decodeCapture, formatGap, packetDetails, unpackSamples} from './usb-parser.js?v=edge-recovery-5';
import {summarizeGaps, validInAckGaps} from './ack-stats.js';

const MAGIC0=0xc7, MAGIC1=0x51;
const CMD={ARM:1,FREEZE:2,STATUS:3,READ:4,CLEAR:5,ACK_TRIGGER:6};
const CMD_NAME={1:'ARM',2:'FREEZE',3:'STATUS',4:'READ',5:'CLEAR',6:'ACK_TRIGGER'};
const $=id=>document.getElementById(id);
const diagnosticLogging=['1','true'].includes(new URLSearchParams(location.search).get('debug')?.toLowerCase());
const ui={connect:$('connect'),disconnect:$('disconnect'),arm:$('arm'),freeze:$('freeze'),download:$('download'),copyCurrent:$('copy-current'),captureExportState:$('capture-export-state'),clear:$('clear'),ackOnly:$('ack-only'),conn:$('connection-state'),status:$('status'),state:$('capture-state'),bus:$('bus-speed'),boundary:$('packet-boundary'),count:$('sample-count'),rate:$('sample-rate'),list:$('packet-list'),packetCount:$('packet-count'),detailTitle:$('detail-title'),details:$('packet-details'),waveform:$('waveform'),waveInfo:$('wave-info'),waveZoomIn:$('wave-zoom-in'),waveZoomOut:$('wave-zoom-out'),waveFit:$('wave-fit')};
let port=null, reader=null, writer=null, readTask=null, statusPoll=null, rx=new Uint8Array(0), pending=null, requestTail=Promise.resolve(), queuedRequests=0, seq=0, captureBytes=null, decoded=[], selectedPacket=null, waveformZoom=1, captureSpeed='FS', captureArmedByGui=false, fetchingCapture=false, autoFetchPending=false, currentCapture=null;
const continuousButton=$('continuous'), statisticsRows=$('statistics'), statisticsCount=$('statistics-count');
let continuousRun=null, manualCommandPending=false, frozenDataAvailable=false, discardPendingFreeze=false;

function diag(event,detail={}){if(diagnosticLogging)console.log(`[usb-capture ${performance.now().toFixed(1)}ms] ${event}`,detail);}
function logException(event,error,detail={}){
  const cause=error?.cause;
  console.error(`[usb-capture ${performance.now().toFixed(1)}ms] ${event}`,{
    ...detail,name:error?.name??'Error',message:error?.message??String(error),stack:error?.stack??null,
    cause:cause instanceof Error?{name:cause.name,message:cause.message,stack:cause.stack}:cause,
  });
}
function hexPreview(bytes,max=20){return Array.from(bytes.subarray(0,max),b=>b.toString(16).padStart(2,'0')).join(' ')+(bytes.length>max?' …':'');}
function setStatus(message,error=false){ui.status.textContent=message;ui.status.title=message;ui.status.classList.toggle('error',error);}
function updateCaptureControls(){
  const manualBusy=!port||!!continuousRun||manualCommandPending||fetchingCapture||autoFetchPending;
  for(const control of [ui.arm,ui.freeze,ui.clear,ui.ackOnly])control.disabled=manualBusy;
  ui.download.disabled=manualBusy||!frozenDataAvailable;
  ui.copyCurrent.disabled=!currentCapture;
  continuousButton.disabled=!port||manualCommandPending||fetchingCapture||autoFetchPending||!!continuousRun?.stopping;
  continuousButton.textContent=continuousRun?.stopping?'Stopping…':continuousRun?'Stop continuous capture':'Start continuous capture';
}
function setConnected(connected){ui.conn.textContent=connected?'Connected':'Disconnected';ui.conn.className=`badge ${connected?'connected':'disconnected'}`;ui.connect.disabled=connected;ui.disconnect.disabled=!connected;updateCaptureControls();if(statusPoll){clearInterval(statusPoll);statusPoll=null;}if(connected)statusPoll=setInterval(()=>{if(port&&!continuousRun&&queuedRequests===0)refreshStatus().catch(e=>logException('status poll failed',e));},1000);}
const u16=(n)=>new Uint8Array([n&255,(n>>>8)&255]);
function u32(n){return new Uint8Array([n&255,(n>>>8)&255,(n>>>16)&255,(n>>>24)&255]);}
function readU16(b,o=0){return b[o]|(b[o+1]<<8);}
function readU32(b,o=0){return (b[o]|(b[o+1]<<8)|(b[o+2]<<16)|(b[o+3]<<24))>>>0;}
function concat(a,b){const c=new Uint8Array(a.length+b.length);c.set(a);c.set(b,a.length);return c;}

async function connect(){
  if(!('serial'in navigator)){setStatus('Web Serial is unavailable. Use Chrome or Edge over HTTPS or localhost.',true);return;}
  try{
    diag('connect: requesting port');
    port=await navigator.serial.requestPort();
    diag('connect: port selected',port.getInfo?.()??{});
    await port.open({baudRate:115200});
    await port.setSignals({dataTerminalReady:true});
    diag('connect: port opened',{baudRate:115200,readable:!!port.readable,writable:!!port.writable});
  }catch(e){logException('connect failed',e);setStatus(`Connection failed: ${e.message}`,true);await disconnect('connect-failed');return;}
  reader=port.readable.getReader();writer=port.writable.getWriter();setConnected(true);setStatus('Connected.');
  readTask=readLoop();
  try{await refreshStatus();}
  catch(e){
    if(statusPoll){clearInterval(statusPoll);statusPoll=null;}
    logException('initial STATUS failed; keeping serial connection open',e,{partialRx:hexPreview(rx,32)});
    const echoedQ=rx.length===1&&rx[0]===0x51;
    setStatus(echoedQ?'STATUS timed out after receiving 0x51 (ASCII “Q”). This usually means the selected serial port is running firmware without packet capture enabled; connection remains open.':'STATUS failed: '+e.message+' Connection remains open for diagnostics.',true);
  }
}
async function disconnect(reason='user'){
  diag('disconnect: begin',{reason,hasPort:!!port,hasReader:!!reader,hasWriter:!!writer,queuedRequests,pending:pending?{cmd:CMD_NAME[pending.cmd],seq:pending.seq}:null});
  const closingReader=reader,closingWriter=writer,closingPort=port;
  captureArmedByGui=false;autoFetchPending=false;
  frozenDataAvailable=false;discardPendingFreeze=false;
  if(continuousRun)continuousRun.stopping=true;
  reader=writer=port=null;
  setConnected(false);
  if(pending){const current=pending;pending=null;clearTimeout(current.timer);diag('disconnect: rejecting pending command',{cmd:CMD_NAME[current.cmd],seq:current.seq});current.reject(new Error('Serial connection closed'));}
  try{if(closingReader){await closingReader.cancel();closingReader.releaseLock();}}catch{}
  try{if(closingWriter){await closingWriter.close();closingWriter.releaseLock();}}catch{}
  try{if(closingPort)await closingPort.close();}catch{}
  rx=new Uint8Array(0);setStatus('Disconnected.');diag('disconnect: complete',{reason});
}
async function readLoop(){
  diag('reader loop started');
  try{while(reader){const {value,done}=await reader.read();if(done){diag('reader returned done');break;}if(value?.length)consume(value);}}
  catch(e){logException('serial reader failed',e,{portStillOpen:!!port});if(port)setStatus(`Serial read failed: ${e.message}`,true);}
}
function consume(chunk){
  diag('serial RX chunk',{bytes:chunk.length,preview:hexPreview(chunk)});
  rx=concat(rx,chunk);
  while(rx.length>=6){
    let at=-1;for(let i=0;i+1<rx.length;i++)if(rx[i]===MAGIC0&&rx[i+1]===MAGIC1){at=i;break;}
    if(at<0){const discard=Math.max(0,rx.length-1);if(discard)diag('discarding non-frame bytes',{bytes:discard,preview:hexPreview(rx.subarray(0,discard),12)});rx=rx.slice(Math.max(0,rx.length-1));return;}
    if(at){diag('discarding bytes before frame',{bytes:at,preview:hexPreview(rx.subarray(0,at),12)});rx=rx.slice(at);}
    if(rx.length<6)return;
    const length=readU16(rx,4);if(length>16384){rx=rx.slice(2);continue;}
    if(rx.length<6+length)return;
    const frame=rx.slice(0,6+length);rx=rx.slice(6+length);
    diag('serial RX frame',{cmd:CMD_NAME[frame[2]&0x7f]??`0x${frame[2].toString(16)}`,response:!!(frame[2]&0x80),seq:frame[3],length,status:frame[6],preview:hexPreview(frame.slice(6),12),matchesPending:!!pending&&frame[2]===(pending.cmd|0x80)&&frame[3]===pending.seq});
    if(pending&&frame[2]===(pending.cmd|0x80)&&frame[3]===pending.seq){const current=pending;pending=null;clearTimeout(current.timer);current.resolve(frame.slice(6));}
  }
}
function request(cmd,payload=new Uint8Array(0)){
  queuedRequests++;
  diag('command queued',{cmd:CMD_NAME[cmd],queuedRequests,payloadLength:payload.length,payload:hexPreview(payload,12)});
  const run=()=>requestOne(cmd,payload);
  const result=requestTail.then(run,run);
  requestTail=result.then(()=>undefined,()=>undefined).finally(()=>{queuedRequests--;diag('command queue advanced',{queuedRequests});});
  return result;
}
async function requestOne(cmd,payload){
  if(!writer){diag('command rejected: no writer',{cmd:CMD_NAME[cmd]});throw new Error('Not connected');}
  if(pending){diag('INVARIANT: command started while another is pending',{cmd:CMD_NAME[cmd],pending:{cmd:CMD_NAME[pending.cmd],seq:pending.seq}});throw new Error('Another command is still in progress');}
  const id=(seq++)&255, frame=new Uint8Array(6+payload.length);frame.set([MAGIC0,MAGIC1,cmd,id,payload.length&255,payload.length>>>8]);frame.set(payload,6);
  const started=performance.now();
  let rejectResponse;
  const response=new Promise((resolve,reject)=>{
    rejectResponse=reject;
    const timer=setTimeout(()=>{if(pending?.seq===id){pending=null;diag('command timeout',{cmd:CMD_NAME[cmd],seq:id,elapsedMs:performance.now()-started,queuedRequests,portOpen:!!port,unparsedRxBytes:rx.length,unparsedRx:hexPreview(rx,32)});reject(new Error('Device response timed out'));}},3000);
    pending={cmd,seq:id,resolve,reject,timer};
  });
  diag('command TX start',{cmd:CMD_NAME[cmd],seq:id,length:payload.length,queuedRequests,portOpen:!!port});
  try{await writer.write(frame);}catch(e){if(pending?.seq===id){clearTimeout(pending.timer);pending=null;}rejectResponse(e);await response;throw e;}
  diag('command TX complete',{cmd:CMD_NAME[cmd],seq:id,elapsedMs:performance.now()-started});
  const body=await response;diag('command completed',{cmd:CMD_NAME[cmd],seq:id,elapsedMs:performance.now()-started,responseLength:body.length,status:body[0]});if(body.length<1)throw new Error('Empty device response');if(body[0]!==0)throw new Error(`Device returned error ${body[0]}`);return body.slice(1);
}
async function refreshStatus({reportEmpty=false}={}){
  const b=await request(CMD.STATUS);if(b.length<19)throw new Error(`Short status response (${b.length} bytes)`);
  const state=b[0],speed=b[1],tailIncomplete=b[2]!==0,sampleCount=readU32(b,3),rate=readU32(b,7),dataSize=readU32(b,11),capacity=readU32(b,15);
  const previousState=ui.state.dataset.value;ui.state.dataset.value=String(state);ui.state.textContent=['Idle','Armed','Frozen'][state]??`State ${state}`;ui.bus.textContent=speed?'Low Speed':'Full Speed';ui.boundary.textContent=tailIncomplete?'May be partial':'Complete';ui.boundary.title=tailIncomplete?'Capture stops at a DMA boundary; the last USB packet may be incomplete.':'The captured tail ended at a USB packet boundary.';ui.count.textContent=`${sampleCount.toLocaleString()} samples · ${dataSize.toLocaleString()} bytes`;ui.rate.textContent=`${(rate/1e6).toFixed(rate>=1e6?1:0)} MHz`;
  frozenDataAvailable=state===2&&dataSize>0;
  const ackTrigger=b.length>=20&&b[19]!==0;if(ui.ackOnly.checked!==ackTrigger){ui.ackOnly.checked=ackTrigger;renderPackets();}
  if(!continuousRun&&state===2){
    const justFrozen=!discardPendingFreeze&&(previousState==='1'||captureArmedByGui);
    captureArmedByGui=false;discardPendingFreeze=false;
    if(dataSize===0&&(justFrozen||reportEmpty)){
      setStatus('No USB transactions were captured. The capture stopped before sampling any packets.');
    }else if(justFrozen){
      setStatus('Capture frozen; fetching automatically…');
      if(!fetchingCapture&&!autoFetchPending){
        autoFetchPending=true;
        queueMicrotask(()=>{autoFetchPending=false;if(port)fetchCapture();});
      }
    }
  }
  updateCaptureControls();
  return {state,speed,tailIncomplete,sampleCount,rate,dataSize,capacity,ackTrigger};
}
async function arm(){try{discardPendingFreeze=false;await request(CMD.ARM);captureArmedByGui=true;const s=await refreshStatus();if(s.state!==2)setStatus(s.ackTrigger?'Armed. Sampling starts at the next non-SOF transaction and freezes after a valid IN DATA+ACK.':'Armed. Sampling starts with the next non-SOF host transaction.');}catch(e){logException('arm failed',e);captureArmedByGui=false;setStatus(e.message,true);}}
async function freeze(){try{discardPendingFreeze=false;await request(CMD.FREEZE);const s=await refreshStatus({reportEmpty:true});if(s.state!==2)setStatus('Save trigger set. Status will update when the capture freezes.');}catch(e){logException('freeze failed',e);setStatus(e.message,true);}}
async function clearCapture(){try{captureArmedByGui=false;await request(CMD.CLEAR);captureBytes=null;decoded=[];selectedPacket=null;currentCapture=null;renderCaptureExportState();renderPackets();drawWaveform();await refreshStatus();setStatus('Capture buffer cleared.');}catch(e){logException('clear failed',e);setStatus(e.message,true);}}
async function fetchCapture(){
  if(fetchingCapture||continuousRun)return;
  fetchingCapture=true;updateCaptureControls();
  try{
    const snapshot=await refreshStatus();
    const data=await readCapture(snapshot);
    showCapture(data,snapshot);
    const ackFilterEmpty=ui.ackOnly.checked&&!decoded.some(p=>p.transactionToken==='IN'&&p.transactionResult==='ACK');
    setStatus(`Loaded ${data.length.toLocaleString()} bytes; decoded ${decoded.length} packet(s).${ackFilterEmpty?' No ACK-completed IN transaction was recognized; showing all decoded packets.':''}`);
  }catch(e){logException('capture fetch failed',e);setStatus(e.message,true);}finally{fetchingCapture=false;updateCaptureControls();}
}
async function readCapture(snapshot,run=null){
  if(snapshot.state!==2)throw new Error('Freeze the capture before reading it.');
  const data=new Uint8Array(snapshot.dataSize);
  for(let offset=0;offset<data.length;){
    if(run?.stopping)return null;
    const count=Math.min(48,data.length-offset);
    const part=await request(CMD.READ,concat(u32(offset),u16(count)));
    if(part.length!==count)throw new Error(`Short capture read at byte ${offset}.`);
    data.set(part,offset);
    offset+=part.length;
    setStatus(`Reading ${offset.toLocaleString()} / ${data.length.toLocaleString()} bytes…`);
  }
  return run?.stopping?null:data;
}
function showCapture(data,snapshot){
  captureBytes=data;
  captureSpeed=snapshot.speed?'LS':'FS';
  let decoderError=null;
  try{decoded=decodeCapture(data,{speed:captureSpeed,sampleRateHz:snapshot.rate});}
  catch(error){decoded=[];decoderError={name:error?.name??'Error',message:error?.message??String(error),stack:error?.stack??null};}
  const hasDiagnostics=!!decoderError||decoded.some(packet=>packet.errors.length);
  const captureRecord={
    savedAt:new Date().toISOString(),
    reason:decoderError?`decoder exception: ${decoderError.message}`:!decoded.length?'no packets decoded':hasDiagnostics?'one or more decoded packets have diagnostics':'all decoded packets passed validation',
    decoderError,
    sampleRateHz:snapshot.rate,
    speedHint:captureSpeed,
    sampleCount:snapshot.sampleCount,
    tailIncomplete:snapshot.tailIncomplete,
    dataSize:data.length,
    decodedPackets:decoded.map(packet=>({index:packet.index,name:packet.name,speed:packet.speed,startSample:packet.startSample,lengthSamples:packet.lengthSamples,bytes:packet.bytes,errors:packet.errors})),
    packedSamplesHex:Array.from(data,b=>b.toString(16).padStart(2,'0')).join(''),
  };
  currentCapture=captureRecord;
  renderCaptureExportState();
  diag('capture decoded',{sampleRateHz:snapshot.rate,captureSpeed,packetCount:decoded.length,decoderError,packets:decoded.map(p=>({name:p.name,speed:p.speed,startSample:p.startSample,errors:p.errors,transaction:p.transactionLabel,result:p.transactionResult}))});
  renderPackets();
}
function renderCaptureExportState(){
  ui.captureExportState.textContent=currentCapture?`Current capture: ${currentCapture.dataSize.toLocaleString()} bytes · ${currentCapture.decodedPackets.length} packet(s) · ${currentCapture.reason}.`:'No capture loaded.';
  updateCaptureControls();
}
async function copyCapture(record){
  if(!record)return;
  const text=JSON.stringify({format:'usb-capture-v1',...record,encoding:'packed hex; each byte contains four chronological 2-bit D+/D− samples, least-significant pair first'},null,2);
  try{
    if(!navigator.clipboard?.writeText)throw new Error('Clipboard API unavailable');
    await navigator.clipboard.writeText(text);
    setStatus(`Copied capture (${record.dataSize.toLocaleString()} raw bytes) to clipboard.`);
  }catch(error){
    const url=URL.createObjectURL(new Blob([text],{type:'application/json'}));
    const link=document.createElement('a');link.href=url;link.download=`usb-capture-${record.savedAt.replace(/[:.]/g,'-')}.json`;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    setStatus(`Clipboard unavailable; downloaded capture JSON (${error.message}).`,true);
  }
}
function manualCommand(action){
  return async ()=>{
    if(!port||continuousRun||manualCommandPending||fetchingCapture||autoFetchPending)return;
    manualCommandPending=true;
    updateCaptureControls();
    try{await action();}finally{manualCommandPending=false;updateCaptureControls();}
  };
}

function renderStatistics(run){
  statisticsCount.textContent=`${run.captures.toLocaleString()} captures · ${run.skipped.toLocaleString()} without a valid IN DATA+ACK`;
  const rows=[];
  for(const [speed,histogram] of Object.entries(run.histograms)){
    const stats=summarizeGaps(histogram);
    if(!stats)continue;
    const gaps=[stats.min,stats.median,stats.mean,stats.p95,stats.max,stats.standardDeviation];
    rows.push(`<tr><td>${speed}</td><td>${stats.count.toLocaleString()}</td>${gaps.map(gap=>`<td>${formatGap(gap,speed)}</td>`).join('')}</tr>`);
  }
  statisticsRows.innerHTML=rows.join('')||'<tr><td colspan="8" class="empty">Waiting for measurements.</td></tr>';
}
async function waitForFrozen(run){
  while(!run.stopping){
    await new Promise(resolve=>setTimeout(resolve,100));
    if(run.stopping)return null;
    const snapshot=await refreshStatus();
    if(snapshot.state===2)return snapshot;
  }
  return null;
}
async function captureContinuously(){
  if(continuousRun){
    continuousRun.stopping=true;
    updateCaptureControls();
    return;
  }
  if(!port||manualCommandPending||fetchingCapture||autoFetchPending)return;

  const run={writer,stopping:false,captures:0,skipped:0,histograms:{FS:new Map(),LS:new Map()}};
  continuousRun=run;
  captureArmedByGui=false;
  discardPendingFreeze=false;
  updateCaptureControls();
  renderStatistics(run);
  let restoreAckTrigger=false,error=null;
  try{
    const initial=await refreshStatus();
    if(!run.stopping&&!initial.ackTrigger){
      restoreAckTrigger=true;
      await request(CMD.ACK_TRIGGER,Uint8Array.of(1));
    }
    if(!run.stopping&&initial.state===1){
      await request(CMD.FREEZE);
      await waitForFrozen(run);
    }
    while(!run.stopping){
      await request(CMD.ARM);
      if(run.stopping)break;
      setStatus('Waiting for an IN DATA+ACK transaction…');
      const snapshot=await waitForFrozen(run);
      if(!snapshot||run.stopping)break;
      const data=await readCapture(snapshot,run);
      if(!data||run.stopping)break;

      run.captures++;
      if(data.length)showCapture(data,snapshot);
      let measurements=0;
      if(data.length){
        for(const [speed,histogram] of Object.entries(run.histograms)){
          const gaps=validInAckGaps(decoded,speed);
          measurements+=gaps.length;
          for(const gap of gaps)histogram.set(gap,(histogram.get(gap)??0)+1);
        }
      }
      if(!measurements)run.skipped++;
      renderStatistics(run);
    }
  }catch(e){
    error=e;
    logException('continuous capture stopped by error',e,{captures:run.captures,skipped:run.skipped,portOpen:!!port});
  }finally{
    if(writer===run.writer){
      try{
        const snapshot=await refreshStatus();
        if(writer===run.writer&&snapshot.state===1){
          discardPendingFreeze=true;
          await request(CMD.FREEZE);
        }
      }catch(e){logException('continuous capture cleanup status failed',e);error??=e;}
      try{
        if(writer===run.writer&&restoreAckTrigger)await request(CMD.ACK_TRIGGER,Uint8Array.of(0));
        if(writer===run.writer)await refreshStatus();
      }catch(e){logException('continuous capture cleanup failed',e);error??=e;}
    }
    continuousRun=null;
    updateCaptureControls();
    if(writer===run.writer)setStatus(error?error.message:'Continuous capture stopped.',!!error);
  }
}
function renderPackets(){
  const acked=decoded.filter(p=>p.transactionToken==='IN'&&p.transactionResult==='ACK');
  const filterHasMatches=ui.ackOnly.checked&&acked.length>0;
  const visible=filterHasMatches?acked:decoded;
  ui.packetCount.textContent=captureBytes?`${visible.length}${filterHasMatches?` / ${decoded.length}`:''} packet${visible.length===1?'':'s'} · ${captureBytes.length.toLocaleString()} bytes${filterHasMatches?' · ACK-completed only':ui.ackOnly.checked&&decoded.length?' · all packets (no ACK-completed IN recognized)':''}`:'No capture loaded';
  if(!decoded.length){selectedPacket=null;ui.list.innerHTML='<tr><td colspan="8" class="empty">No packets decoded. Check the sample encoding and capture speed.</td></tr>';drawWaveform();return;}
  if(!visible.includes(selectedPacket))selectedPacket=visible[0];
  ui.list.innerHTML=visible.map(p=>`<tr data-index="${decoded.indexOf(p)}" class="${p.errors.length?'error-row':''}${p===selectedPacket?' selected':''}"><td>${p.index}</td><td>${p.startTimeUs.toFixed(2)}</td><td>${formatGap(p.gapBitTime,p.speed,p.gapTimeUs)}</td><td>${p.lengthBitTime.toFixed(2)} ${p.speed} bt</td><td>${p.speed} · ${p.type} · ${p.name}</td><td>${p.addr===null?'—':`${p.addr}${p.endpoint===null?'':` / ${p.endpoint}`}`}</td><td>${p.payload.length?bytesHex(p.payload):'—'}</td><td>${p.errors.length?p.errors.join(', '):p.transactionResult??'OK'}</td></tr>`).join('');
  ui.list.querySelectorAll('tr[data-index]').forEach(row=>row.addEventListener('click',()=>selectPacket(decoded[Number(row.dataset.index)])));
  selectPacket(selectedPacket);
}
function selectPacket(packet){
  selectedPacket=packet;
  ui.list.querySelectorAll('tr[data-index]').forEach(row=>row.classList.toggle('selected',decoded[Number(row.dataset.index)]===packet));
  ui.detailTitle.textContent=`Packet ${packet.index} · ${packet.name}`;
  ui.details.textContent=packetDetails(packet,{speed:captureSpeed});
  drawWaveform(packet);
}
function drawWaveform(packet=selectedPacket){
  const canvas=ui.waveform, height=240, dpr=Math.min(window.devicePixelRatio||1,2);
  if(!captureBytes?.length||!packet){
    const width=Math.max(900,ui.waveform.parentElement.clientWidth);
    canvas.style.width=`${width}px`;canvas.width=Math.ceil(width*dpr);canvas.height=height*dpr;
    const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#101720';ctx.fillRect(0,0,width,height);
    ctx.fillStyle='#91a0b2';ctx.font='14px system-ui';ctx.fillText(captureBytes?'No decoded packet selected.':'Fetch a capture and select a packet.',24,36);
    ui.waveInfo.textContent=captureBytes?'No decoded packet selected.':'Waiting for capture';return;
  }
  const samples=unpackSamples(captureBytes);
  const transaction=packet.transactionId===undefined?[packet]:decoded.filter(p=>p.transactionId===packet.transactionId);
  const transactionStart=Math.min(...transaction.map(p=>p.startSample));
  const transactionEnd=Math.max(...transaction.map(p=>p.endSample));
  const padding=32; // 8 bit times of idle context on each side
  const start=Math.max(0,transactionStart-padding),end=Math.min(samples.length,transactionEnd+padding);
  const sampleSpan=Math.max(1,end-start);
  const fitScale=Math.min(3,1120/sampleSpan);
  const width=Math.max(900,Math.min(16000,Math.ceil(76+sampleSpan*fitScale*waveformZoom)));
  const plotLeft=64,plotRight=width-20,plotWidth=plotRight-plotLeft,scale=plotWidth/sampleSpan;
  canvas.style.width=`${width}px`;canvas.width=Math.ceil(width*dpr);canvas.height=height*dpr;
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#101720';ctx.fillRect(0,0,width,height);
  ui.waveInfo.textContent=`${transaction[0]?.transactionLabel??packet.name} · ${packet.speed} · ${packet.samplesPerBit} samples/bit · ${waveformZoom}×`;

  const colors={Token:'rgba(103,166,255,.16)',Data:'rgba(103,213,187,.15)',Handshake:'rgba(241,197,107,.18)',Special:'rgba(180,150,220,.16)'};
  for(const p of transaction){
    const x0=plotLeft+(Math.max(start,p.startSample)-start)*scale,x1=plotLeft+(Math.min(end,p.endSample)-start)*scale;
    if(x1<=x0)continue;
    ctx.fillStyle=colors[p.type]??'rgba(145,160,178,.12)';ctx.fillRect(x0,34,x1-x0,34);
    ctx.strokeStyle=p===packet?'#e6edf3':'rgba(230,237,243,.35)';ctx.lineWidth=p===packet?1.5:1;ctx.strokeRect(x0+.5,34.5,Math.max(0,x1-x0-1),33);
    const token=p.addr===null?'':` ${p.addr}/${p.endpoint??0}`;
    const payload=p.type==='Data'&&p.payload.length?` ${bytesHex(p.payload.slice(0,3))}${p.payload.length>3?'…':''}`:'';
    const error=p.errors.length?' !':'';
    const label=`${p.name}${token}${payload}${error}`;
    ctx.fillStyle='#e6edf3';ctx.font='11px ui-monospace,monospace';ctx.textBaseline='middle';ctx.fillText(label,x0+3,51,Math.max(8,x1-x0-6));
  }

  const topHigh=91,topLow=113,bottomHigh=145,bottomLow=167;
  ctx.font='12px system-ui';ctx.textBaseline='middle';ctx.fillStyle='#67d5bb';ctx.fillText('D+',15,(topHigh+topLow)/2);ctx.fillStyle='#82aaff';ctx.fillText('D−',15,(bottomHigh+bottomLow)/2);
  ctx.strokeStyle='rgba(145,160,178,.15)';ctx.lineWidth=1;
  for(const y of [topHigh,topLow,bottomHigh,bottomLow]){ctx.beginPath();ctx.moveTo(plotLeft,y+.5);ctx.lineTo(plotRight,y+.5);ctx.stroke();}
  const samplesPerBit=packet.samplesPerBit??4;
  const bitsInView=sampleSpan/samplesPerBit,tickBits=Math.max(1,Math.ceil(bitsInView/12));
  const firstTick=Math.ceil(start/samplesPerBit/tickBits)*tickBits;
  ctx.font='10px ui-monospace,monospace';ctx.textBaseline='top';ctx.fillStyle='#91a0b2';ctx.strokeStyle='rgba(145,160,178,.22)';
  for(let bt=firstTick;bt<=end/samplesPerBit;bt+=tickBits){const x=plotLeft+(bt*samplesPerBit-start)*scale;ctx.beginPath();ctx.moveTo(x+.5,72);ctx.lineTo(x+.5,184);ctx.stroke();ctx.fillText(`${bt}`,x+2,190);}
  ctx.fillText('bit time',plotRight-48,214);
  const drawLane=(bit,high,low,color)=>{
    ctx.strokeStyle=color;ctx.lineWidth=1.7;ctx.beginPath();
    let previous=(samples[start]>>bit)&1;ctx.moveTo(plotLeft,previous?high:low);
    for(let i=start+1;i<end;i++){const x=plotLeft+(i-start)*scale;ctx.lineTo(x,previous?high:low);const current=(samples[i]>>bit)&1;if(current!==previous){ctx.lineTo(x,current?high:low);previous=current;}}
    ctx.lineTo(plotRight,previous?high:low);ctx.stroke();
  };
  drawLane(0,topHigh,topLow,'#67d5bb');drawLane(1,bottomHigh,bottomLow,'#82aaff');
}
function bytesHex(bytes){const str=bytes.map(b=>b.toString(16).padStart(2,'0')).join(' ');return str.length>40?str.slice(0,37)+'…':str;}

ui.waveZoomIn.addEventListener('click',()=>{waveformZoom=Math.min(8,waveformZoom*2);drawWaveform();});ui.waveZoomOut.addEventListener('click',()=>{waveformZoom=Math.max(1,waveformZoom/2);drawWaveform();});ui.waveFit.addEventListener('click',()=>{waveformZoom=1;drawWaveform();});
window.addEventListener('resize',()=>drawWaveform());
ui.connect.addEventListener('click',connect);ui.disconnect.addEventListener('click',()=>disconnect('user'));ui.arm.addEventListener('click',manualCommand(arm));ui.freeze.addEventListener('click',manualCommand(freeze));ui.clear.addEventListener('click',manualCommand(clearCapture));ui.download.addEventListener('click',fetchCapture);ui.copyCurrent.addEventListener('click',()=>copyCapture(currentCapture));ui.ackOnly.addEventListener('change',renderPackets);
continuousButton.addEventListener('click',captureContinuously);
ui.ackOnly.addEventListener('change',manualCommand(async()=>{const enabled=ui.ackOnly.checked;try{await request(CMD.ACK_TRIGGER,new Uint8Array([enabled?1:0]));renderPackets();setStatus(enabled?'ACK trigger enabled; Arm / Resume will stop on a valid IN DATA+ACK.':'ACK trigger disabled; Save now is manual.');}catch(e){logException('ACK trigger update failed',e);ui.ackOnly.checked=!enabled;renderPackets();setStatus(e.message,true);}}));
navigator.serial?.addEventListener('disconnect',event=>{diag('Web Serial disconnect event',{portInfo:event.target?.getInfo?.()??{}});disconnect('webserial-disconnect-event');});
window.addEventListener('error',event=>{
  const error=event.error??new Error(event.message||'Uncaught browser error');
  logException('uncaught browser error',error,{filename:event.filename,lineno:event.lineno,colno:event.colno});
});
window.addEventListener('unhandledrejection',event=>logException('unhandled promise rejection',event.reason));

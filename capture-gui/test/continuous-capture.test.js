import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {formatGap} from '../usb-parser.js';
import {summarizeGaps, validInAckGaps} from '../ack-stats.js';

const source = readFileSync(new URL('../app.js', import.meta.url), 'utf8')
  .replace(/^import .*\n/gm, '');

function mockCapture(snapshots, {stopDuringRead = false} = {}) {
  const elements = new Map();
  const get = id => {
    if (!elements.has(id)) elements.set(id, {dataset: {}, classList: {toggle() {}}, addEventListener() {}});
    return elements.get(id);
  };
  const operations = [];
  let capture, stop, armed = false, ackTrigger = false;
  const context = vm.createContext({
    document: {getElementById: get}, location: {search: ''}, URLSearchParams,
    window: {addEventListener() {}}, navigator: {}, Uint8Array, queueMicrotask,
    setTimeout: queueMicrotask, formatGap, summarizeGaps, validInAckGaps,
    async serialRequest(cmd, payload) {
      switch (cmd) {
        case 1:
          operations.push('ARM');
          armed = true;
          capture = snapshots.shift();
          if (!capture) await stop();
          break;
        case 2:
          operations.push('FREEZE');
          armed = false;
          break;
        case 3: {
          const status = new Uint8Array(20);
          const report = new DataView(status.buffer);
          if (armed && capture) armed = false;
          status[0] = armed ? 1 : 2;
          status[1] = capture?.speed ?? 0;
          status[19] = ackTrigger;
          report.setUint32(11, capture?.size ?? 0, true);
          return status;
        }
        case 4: {
          const read = new DataView(payload.buffer);
          const offset = read.getUint32(0, true);
          const count = read.getUint16(4, true);
          operations.push(`READ ${offset}+${count}`);
          if (stopDuringRead) await stop();
          return new Uint8Array(count);
        }
        case 6:
          ackTrigger = !!payload[0];
          operations.push(`ACK_TRIGGER ${payload[0]}`);
          break;
        default:
          throw new Error(`Unexpected command ${cmd}`);
      }
    },
    examplePackets(lowSpeed) {
      const speed = lowSpeed ? 'LS' : 'FS';
      return [
        {name:'IN',speed,bytes:[0x69,0,0],errors:[],transactionId:0},
        {name:'DATA0',speed,errors:[],transactionId:0},
        {name:'ACK',speed,errors:[],transactionId:0,dataAckGapBitTime:lowSpeed?4:11.75},
      ];
    },
  });
  vm.runInContext(source + `
    port={};writer={};request=serialRequest;renderPackets=()=>{};
    showCapture=(data,snapshot)=>{decoded=examplePackets(snapshot.speed);};
  `, context);
  stop = () => vm.runInContext('captureContinuously()', context);
  return {start: stop, operations, get};
}

test('reads each snapshot before re-arming and preserves split statistics after stop', async () => {
  const {start, operations, get} = mockCapture([{size:52}, {size:0}, {size:1,speed:1}]);
  await start();
  assert.deepEqual(operations, [
    'ACK_TRIGGER 1', 'ARM', 'READ 0+48', 'READ 48+4', 'ARM', 'ARM', 'READ 0+1',
    'ARM', 'FREEZE', 'ACK_TRIGGER 0',
  ]);
  assert.match(get('statistics-count').textContent, /3 captures · 1 without/);
  assert.match(get('statistics').innerHTML, /<td>FS<\/td><td>1<\/td>.*11\.75 bt \/ 0\.979 µs/);
  assert.match(get('statistics').innerHTML, /<td>LS<\/td><td>1<\/td>.*4\.00 bt \/ 2\.667 µs/);
  assert.equal(get('status').textContent, 'Continuous capture stopped.');
});

test('stop during a fetch discards the partial read and issues no further arm', async () => {
  const {start, operations, get} = mockCapture([{size:52}], {stopDuringRead:true});
  await start();
  assert.deepEqual(operations, ['ACK_TRIGGER 1', 'ARM', 'READ 0+48', 'ACK_TRIGGER 0']);
  assert.match(get('statistics-count').textContent, /^0 captures/);
});

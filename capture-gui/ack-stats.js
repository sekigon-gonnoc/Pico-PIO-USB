export function validInAckGaps(packets, speed) {
  const gaps = [];
  for (let i = 2; i < packets.length; i++) {
    const ack = packets[i];
    if (ack.name !== 'ACK' || (speed && ack.speed !== speed) || ack.errors.length) continue;

    // A hub may need another full-speed PRE before the host's low-speed ACK.
    const pre = packets[i - 1].name === 'PRE' ? packets[i - 1] : null;
    if (pre && (pre.transactionId !== ack.transactionId || pre.errors.length)) continue;
    const dataIndex = i - (pre ? 2 : 1);
    const token = packets[dataIndex - 1], data = packets[dataIndex];
    if (token?.name !== 'IN' || token.bytes.length !== 3 ||
        (data.name !== 'DATA0' && data.name !== 'DATA1') ||
        token.transactionId !== ack.transactionId || data.transactionId !== ack.transactionId ||
        token.speed !== ack.speed || data.speed !== ack.speed ||
        token.errors.length || data.errors.length) continue;
    const gap = ack.dataAckGapBitTime;
    if (Number.isFinite(gap) && gap >= 0) gaps.push(gap);
  }
  return gaps;
}

export function summarizeGaps(histogram) {
  const bins = [...histogram].sort(([a], [b]) => a - b);
  if (!bins.length) return null;

  const count = bins.reduce((sum, [, frequency]) => sum + frequency, 0);
  const mean = bins.reduce((sum, [gap, frequency]) => sum + gap * frequency, 0) / count;
  const variance = bins.reduce((sum, [gap, frequency]) => sum + (gap - mean) ** 2 * frequency, 0) / count;
  function percentile(fraction) {
    const rank = (count - 1) * fraction;
    let remaining = Math.floor(rank);
    for (let i = 0; i < bins.length; i++) {
      const [gap, frequency] = bins[i];
      if (remaining < frequency) {
        const next = remaining === frequency - 1 && i + 1 < bins.length ? bins[i + 1][0] : gap;
        return gap + (next - gap) * (rank % 1);
      }
      remaining -= frequency;
    }
  }
  return {
    count, min: bins[0][0], mean, median: percentile(0.5), p95: percentile(0.95),
    max: bins.at(-1)[0], standardDeviation: Math.sqrt(variance),
  };
}

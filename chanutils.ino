
void dumpOutputChannels() {
  Serial.println("Output channels:");
  for (int i = 0; i < N_OUTPUT_CHANNELS; i++) {
    char buf[80];
    sprintf(buf, "%d: R: %d\tRw: %d;\tG: %d\tGw: %d;\tB: %d\tBw: %d;\tLL: %d", i, 
      outputChannelFreqs[i].fslot_r, outputChannelFreqs[i].fwidth_r,
      outputChannelFreqs[i].fslot_g, outputChannelFreqs[i].fwidth_g,
      outputChannelFreqs[i].fslot_b, outputChannelFreqs[i].fwidth_b,
      outputChannelFreqs[i].showLines);
    Serial.println(buf);
  }
}

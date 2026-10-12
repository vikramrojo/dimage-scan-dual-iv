import ghidra.app.script.GhidraScript;
import ghidra.program.model.mem.*;
import ghidra.program.model.address.*;
public class DumpMem extends GhidraScript {
  public void run() throws Exception {
    Memory m = currentProgram.getMemory();
    for (String s : getScriptArgs()) {
      String[] p = s.split(":"); Address a = toAddr(Long.parseLong(p[1],16));
      int n = p.length > 2 ? Integer.parseInt(p[2]) : 64;
      if (p[0].equals("p")) { a = toAddr(m.getInt(a) & 0xffffffffL); }
      byte[] b = new byte[n]; m.getBytes(a, b);
      StringBuilder sb = new StringBuilder(s + " @" + a + ": ");
      for (byte x : b) sb.append(String.format("%02x", x));
      sb.append("\n   doubles:");
      for (int i = 0; i + 8 <= n; i += 8) { long v = 0; for (int j = 0; j < 8; j++) v = (v << 8) | (b[i+j] & 0xff); sb.append(" " + Double.longBitsToDouble(v)); }
      sb.append("\n   floats:");
      for (int i = 0; i + 4 <= n; i += 4) { int v = 0; for (int j = 0; j < 4; j++) v = (v << 8) | (b[i+j] & 0xff); sb.append(" " + Float.intBitsToFloat(v)); }
      println(sb.toString());
    }
  }
}

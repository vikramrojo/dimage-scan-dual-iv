import ghidra.app.script.GhidraScript;
import ghidra.program.model.mem.*;
import ghidra.program.model.address.*;
public class FixToc extends GhidraScript {
  public void run() throws Exception {
    Memory m = currentProgram.getMemory(); int n = 0;
    for (MemoryBlock b : m.getBlocks()) {
      if (!b.isExecute()) continue;
      for (Address a = b.getStart(); a.compareTo(b.getEnd()) < 0; a = a.add(4)) {
        if (m.getInt(a) == 0x80410014) { m.setInt(a, 0x60000000); n++; }
      }
    }
    println("patched r2 reloads: " + n);
  }
}

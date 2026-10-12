import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.*;
import ghidra.program.model.listing.*;
import ghidra.program.model.symbol.SourceType;
import java.io.*;
public class CreateFuncs extends GhidraScript {
  public void run() throws Exception {
    String fn = getScriptArgs()[0];
    BufferedReader r = new BufferedReader(new FileReader(fn)); String l; int n=0;
    while ((l = r.readLine()) != null) {
      String[] p = l.split(" ", 2);
      Address a = toAddr(Long.parseLong(p[0], 16));
      Function f = getFunctionAt(a);
      if (f == null) { disassemble(a); f = createFunction(a, null); if (f != null) n++; }
      if (f != null && f.getName().startsWith("FUN_")) { try { f.setName(p[1].replace(' ', '_'), SourceType.USER_DEFINED); } catch (Exception e) {} }
    }
    println("created " + n);
  }
}

import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.*;
import ghidra.program.model.data.*;
import ghidra.program.model.symbol.SourceType;
import ghidra.app.util.parser.FunctionSignatureParser;
import ghidra.app.cmd.function.ApplyFunctionSignatureCmd;
public class FixSigs extends GhidraScript {
  void apply(Function f, String sig) throws Exception {
    FunctionSignatureParser p = new FunctionSignatureParser(currentProgram.getDataTypeManager(), null);
    FunctionDefinitionDataType d = p.parse(f.getSignature(), sig);
    new ApplyFunctionSignatureCmd(f.getEntryPoint(), d, SourceType.USER_DEFINED).applyTo(currentProgram);
    println("fixed " + f.getName());
  }
  public void run() throws Exception {
    FunctionIterator it = currentProgram.getFunctionManager().getFunctions(true);
    while (it.hasNext()) {
      Function f = it.next(); String n = f.getName();
      if (f.isThunk()) continue;
      if (n.contains("AVAllocateMem")) apply(f, "void * AVAllocateMem(uint size)");
      else if (n.contains("AVFreeMem")) apply(f, "void AVFreeMem(void * p)");
      else if (n.contains("SwapWORD")) apply(f, "ushort SwapWORD(ushort v)");
      else if (n.contains("SwapDWORD")) apply(f, "uint SwapDWORD(uint v)");
      else if (n.equals("NewPtr") || n.equals("NewPtrClear")) apply(f, "void * " + n + "(uint size)");
    }
  }
}

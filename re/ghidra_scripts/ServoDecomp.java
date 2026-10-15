// Post-analysis: decompile every function into one C file plus a function list.
//@category Servo
import ghidra.app.decompiler.*;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.*;
import java.io.*;

public class ServoDecomp extends GhidraScript {
    public void run() throws Exception {
        String out = getScriptArgs().length > 0 ? getScriptArgs()[0] : "C:\\tools\\re\\out";
        new File(out).mkdirs();
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        try (PrintWriter c = new PrintWriter(new FileWriter(out + "\\decomp.c"));
             PrintWriter l = new PrintWriter(new FileWriter(out + "\\functions.txt"))) {
            for (Function f : currentProgram.getFunctionManager().getFunctions(true)) {
                l.printf("%s %6d %s%n", f.getEntryPoint(), f.getBody().getNumAddresses(), f.getName());
                DecompileResults r = di.decompileFunction(f, 60, monitor);
                c.printf("// ==== %s @ %s ====%n", f.getName(), f.getEntryPoint());
                c.println(r.decompileCompleted() ? r.getDecompiledFunction().getC() : "// decompile failed: " + r.getErrorMessage());
            }
        }
        di.dispose();
    }
}

// Pre-analysis setup for LCM32F037 raw flash dump: Thumb mode, vector table, memory blocks.
//@category Servo
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.*;
import ghidra.program.model.lang.Register;
import ghidra.program.model.listing.*;
import ghidra.program.model.mem.*;
import ghidra.program.model.symbol.SourceType;
import java.math.BigInteger;

public class ServoPre extends GhidraScript {
    static final long FLASH = 0x08000000L, CODE_END = 0x08004E54L;
    static final String[] SYS = {"Initial_SP", "Reset", "NMI", "HardFault", "Res4", "Res5", "Res6",
        "Res7", "Res8", "Res9", "Res10", "SVCall", "Res12", "Res13", "PendSV", "SysTick"};

    void block(String name, long start, long size, boolean vol) throws Exception {
        Memory mem = currentProgram.getMemory();
        Address a = toAddr(start);
        if (mem.getBlock(a) != null) return;
        MemoryBlock b = mem.createUninitializedBlock(name, a, size, false);
        b.setRead(true); b.setWrite(true); b.setExecute(false); b.setVolatile(vol);
    }

    public void run() throws Exception {
        // Read-only flash lets the decompiler fold literal-pool constants into real addresses
        currentProgram.getMemory().getBlock(toAddr(FLASH)).setWrite(false);
        block("SRAM", 0x20000000L, 0x1000, false);
        block("SYSMEM", 0x1FFFF000L, 0x1000, false);
        block("PERIPH", 0x40000000L, 0x10000000L, true);
        block("PERIPH2", 0x50000000L, 0x10000000L, true);
        block("SCS", 0xE0000000L, 0x100000, true);

        Register tmode = currentProgram.getRegister("TMode");
        currentProgram.getProgramContext().setValue(tmode, toAddr(FLASH), toAddr(CODE_END - 1), BigInteger.ONE);

        vectors(FLASH, "bl_");
        vectors(0x08001000L, "");
    }

    void vectors(long table, String prefix) throws Exception {
        Memory mem = currentProgram.getMemory();
        for (int i = 0; i < 48; i++) {
            Address va = toAddr(table + i * 4);
            clearListing(va, va.add(3));
            createDWord(va);
            String vname = i < 16 ? SYS[i] : "IRQ" + (i - 16);
            createLabel(va, prefix + "vec_" + vname, true, SourceType.USER_DEFINED);
            if (i == 0) continue;
            long v = mem.getInt(va) & 0xffffffffL;
            if (v < FLASH || v >= CODE_END) continue;
            Address fa = toAddr(v & ~1L);
            disassemble(fa);
            Function f = getFunctionAt(fa);
            if (f == null) f = createFunction(fa, prefix + vname + "_Handler");
            else if (f.getName().startsWith("FUN_")) f.setName(prefix + vname + "_Handler", SourceType.USER_DEFINED);
            currentProgram.getSymbolTable().addExternalEntryPoint(fa);
        }
    }
}

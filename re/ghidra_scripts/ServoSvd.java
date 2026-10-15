// Label peripheral registers from a CMSIS SVD file (handles derivedFrom at peripheral level).
//@category Servo
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.symbol.SourceType;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.*;
import java.io.File;
import java.util.*;

public class ServoSvd extends GhidraScript {
    static long num(String s) {
        s = s.trim().toLowerCase();
        return s.startsWith("0x") ? Long.parseLong(s.substring(2), 16) : Long.parseLong(s);
    }

    static String text(Element e, String tag) {
        NodeList l = e.getElementsByTagName(tag);
        return l.getLength() > 0 ? l.item(0).getTextContent() : null;
    }

    public void run() throws Exception {
        String path = getScriptArgs().length > 0 ? getScriptArgs()[0] : "C:\\tools\\re\\LCM32F037.svd";
        Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(new File(path));
        Map<String, Element> byName = new HashMap<>();
        NodeList ps = doc.getElementsByTagName("peripheral");
        for (int i = 0; i < ps.getLength(); i++) {
            Element p = (Element) ps.item(i);
            byName.put(text(p, "name"), p);
        }
        int n = 0;
        for (int i = 0; i < ps.getLength(); i++) {
            Element p = (Element) ps.item(i);
            String pname = text(p, "name");
            long base = num(text(p, "baseAddress"));
            Element src = p;
            if (p.hasAttribute("derivedFrom")) src = byName.get(p.getAttribute("derivedFrom"));
            createLabel(toAddr(base), pname, true, SourceType.IMPORTED);
            NodeList regs = src.getElementsByTagName("register");
            for (int j = 0; j < regs.getLength(); j++) {
                Element r = (Element) regs.item(j);
                String off = text(r, "addressOffset");
                if (off == null) continue;
                Address a = toAddr(base + num(off));
                if (currentProgram.getMemory().getBlock(a) == null) continue;
                createLabel(a, pname + "_" + text(r, "name"), true, SourceType.IMPORTED);
                n++;
            }
        }
        println("SVD labels: " + n);
    }
}

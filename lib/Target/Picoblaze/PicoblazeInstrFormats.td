//===----------------------------------------------------------------------===//
//  Describe Picoblaze instructions format
//
//  CPU INSTRUCTION FORMATS
//  17:12 op
//===----------------------------------------------------------------------===//

// Format specifies the encoding used by the instruction.  This is part of the
// ad-hoc solution used to emit machine instruction encodings by our machine
// code emitter.
class Format<bits<3> val> {
  bits<3> Value = val;
}

def FrmPsu     : Format<0>;
def FrmALU     : Format<1>;
def FrmALUI    : Format<2>;
def FrmSHIFT   : Format<3>;
def FrmBRANCH  : Format<4>;

// Generic Picoblaze Format
class PicoblazeInst<dag outs, dag ins, string asmstr, list<dag> pattern,
                Format f>: Instruction
{
  field bits<18> Inst;
  Format Form = f;

  let Namespace = "Picoblaze";

  let Size = 4;

  bits<6> Opcode = 0;

  // Top 8 bits are the 'opcode' field
  let Inst{17-12} = Opcode;

  let OutOperandList = outs;
  let InOperandList  = ins;

  let AsmString   = asmstr;
  let Pattern     = pattern;


  //
  // Attributes specific to Picoblaze instructions...
  //
  bits<3> FormBits = Form.Value;

  // TSFlags layout should be kept in sync with PicoblazeInstrInfo.h.
  let TSFlags{2-0}   = FormBits;

  let DecoderNamespace = "Picoblaze";


}

//===----------------------------------------------------------------------===//
// Format FrmALU instruction class in Picoblaze : <|opcode|ra|rb|rc|cx|>
//===----------------------------------------------------------------------===//

class FALU<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern>:
      PicoblazeInst<outs, ins, asmstr, pattern,  FrmALU>
{
  bits<4>  sX;
  bits<4>  sY;
  
  let Opcode = op;

  let Inst{11-8} = sX;
  let Inst{7-4} = sY;
  let Inst{3-0} = 0;
}

class FALUI<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern>:
      PicoblazeInst<outs, ins, asmstr, pattern,  FrmALUI>
{
  bits<4>  sX;
  bits<8>  kk;
  
  let Opcode = op;

  let Inst{11-8} = sX;
  let Inst{7-0}  = kk;
}

class FSHIFT<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern>:
      PicoblazeInst<outs, ins, asmstr, pattern,  FrmSHIFT>
{
  bits<4>  sX;
  bits<4>  shiftType;
  
  let Opcode = op;

  let Inst{11-8} = sX;
  let Inst{7-4}  = shiftType;
  let Inst{3-0} = 0;
}


class FBRANCH<bits<6> op, dag outs, dag ins, string asmstr,
         list<dag> pattern>:
      PicoblazeInst<outs, ins, asmstr, pattern,  FrmBRANCH>
{
  bits<2>  cond;
  let Opcode = op;

  let Inst{11-10} = cond;
  let Inst{9-0} = 0;
}


class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : PicoblazeInst<outs, ins, asmstr, pattern, FrmPsu> {
  let Pattern = pattern;
  let Inst{15-0} = 0;
}



#include "trapframe.h"
#include "riscv-gdb-stub.h"
#include "encoding.h"



trapframe_t* __attribute__((weak)) trap_handler(trapframe_t *ptf)
{

    if (ptf->cause & 0x80000000) {
      // place interrupt handler here...
      return ptf;
    }  else {
       return handle_exception(ptf);
    }
}


extern void __trap();

t_ptrapfuntion gdb_initDebugger(int set_mtvec)
{
  if (set_mtvec) {
     write_csr(mtvec,__trap);
  }
  return handle_exception;
}


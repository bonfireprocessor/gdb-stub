#include <stdio.h>
#include "console.h"

#include "riscv-gdb-stub.h"

void do_increment(volatile int *px)
{
  (*px)++;
}

#define BAUDRATE 115200

#define SIZE 10

uint64_t  test[SIZE];

void initArray()
{
int i;

  for(i=0;i<SIZE;i++) test[i]=i*i;

}


void insertLastAt(int pos)
// Inserts Last element of the array at pos
{

uint64_t *p;

   for(p=&test[SIZE-1];p>&test[pos];p--) {
       *p = *(p-1);
   }
   test[pos]=test[SIZE-1];
}



void arrayTest()
{
int i;

  initArray();
  for(i=0;i<SIZE;i++) printk("%4llu ",test[i]);
  insertLastAt(3);
  printk("\n");
  for(i=0;i<SIZE;i++) printk("%4llu ",test[i]);
  gdb_breakpoint();
}


void main()
{
volatile int i=0;

  printk("Run with baudrate %d\n",BAUDRATE);
  gdb_setup_interface(BAUDRATE);
  gdb_initDebugger(1);


  arrayTest();

  while(1) {

    do_increment(&i);
    if ((i % 10000)==0 ) printk("*");
  }
}

// Test 82C55A using Arduino

// Connections:
// 82C55A            Arduino
// ----------------------------------------
// D0                D0(0)
// D1                D1(1)
// D2                D2(2)
// D3                D3(3)
// D4                D4(4)
// D5                D5(5)
// D6                D6(6)
// D7                D7(7)
// A0                C0(14)
// A1                C1(15)
// -CS               C2(16)
// -RD               C3(17)
// -WR               C4(18)
// RESET             C5(19)

// The outputs of port A are used to drive LEDs, using
// ULN2003A or ULN2803A transistor arrays (rather than
// driving the LEDs directly via the 82C55A output pins,
// which can only source or sink 2.5 mA according to the
// data sheet.)

// An 8 position DIP switch is connected to the pins of
// port B.  When open, the switches are pulled low
// (to ground) using 4k7 resistors.  When closed, the
// switches are pulled high (to VCC.)

// control pin bits (in port C)
#define A0           (1 << 0)
#define A1           (1 << 1)
#define CS           (1 << 2) // active low
#define RD           (1 << 3) // active low
#define WR           (1 << 4) // active low
#define RESET        (1 << 5)

// Mask for address bits
#define ADDR_MASK    0x03

// Set the register address
#define set_addr(reg) \
do { PORTC = (PORTC & ~ADDR_MASK) | ((reg) & ADDR_MASK); } while (0)

// 82C55A registers
#define REG_PORTA    0
#define REG_PORTB    1
#define REG_PORTC    2
#define REG_CTRL     3

// Control word bits
// Note that "group A" is port A and port C upper,
// and "group B" is port B and port C lower

#define CTRL_MODESET                0x80     // set mode, if clear set/reset bit

#define CTRL_GROUPA_MODE0           0        // group A: set mode 0
#define CTRL_GROUPA_MODE1           0x20     // group A: set mode 1
#define CTRL_GROUPA_MODE2           0x40     // group A: set mode 2
#define CTRL_GROUPA_PORTA_IN        0x10     // group A: configure port A for input
#define CTRL_GROUPA_PORTA_OUT       0        // group A: configure port A for output
#define CTRL_GROUPA_PORTC_UPPER_IN  0x08     // group A: configure port C upper for input
#define CTRL_GROUPA_PORTC_UPPER_OUT 0        // group A: configure port C upper for output

#define CTRL_GROUPB_MODE0           0        // group B: set mode 0
#define CTRL_GROUPB_MODE1           0x04     // group B: set mode 1
#define CTRL_GROUPB_PORTB_IN        0x02     // group B: configure port B for input
#define CTRL_GROUPB_PORTB_OUT       0        // group B: configure port B for output
#define CTRL_GROUPB_PORTC_LOWER_IN  0x01     // group B: configure port C lower for input
#define CTRL_GROUPB_PORTC_LOWER_OUT 0        // group B: configure port C lower for output

// limit speed of bus operations
#define bus_delay() delayMicroseconds(2)

// Initialize the 82C55A.
void i82c55a_init(void) {
  // in quiescent state, configure data port as all inputs
  // with pullups enabled
  DDRD = 0;
  PORTD = 0xff;

  // all pins on control port are configured for output
  DDRC = 0xff;

  // initially, A0/A1 are low, CS, RD, and WR are
  // not asserted, RESET is asserted
  PORTC = CS | RD | WR | RESET;

  // wait a bit
  delay(500);

  // deassert reset
  PORTC &= ~RESET;
}

// Write to an 82C55A register.
// Parameters:
//   reg - the register (0..3)
//   val - the value to write
void i82c55a_write(uint8_t reg, uint8_t val) {
  // configure the data port for output
  DDRD = 0xff;

  // assert data
  PORTD = val;

  // assert register address
  set_addr(reg);

  // assert chip select
  PORTC &= ~CS;
  bus_delay();

  // strobe the write signal
  PORTC &= ~WR;
  bus_delay();
  PORTC |= WR;
  bus_delay();

  // deassert chip select
  PORTC |= CS;
  bus_delay();

  // configure data port as input again
  DDRD = 0;
  PORTD = 0xff;
}

// Read from the 82C55A.
// Parameters:
//   reg - the register (0..3)
// Returns:
//   the value read
uint8_t i82c55a_read(uint8_t reg) {
  // assert register address
  set_addr(reg);

  // assert chip select
  PORTC &= ~CS;
  bus_delay();

  // assert read signal
  PORTC &= ~RD;
  bus_delay();

  // read the data
  uint8_t val = PIND;
  bus_delay();     // not sure if strictly necessary, but can't hurt

  // deassert read signal
  PORTC |= RD;
  bus_delay();

  // deassert chip select
  PORTC |= CS;
  bus_delay();

  return val;
}

void setup() {
  i82c55a_init();

  // put all ports in mode 0, configure ports A and C for output,
  // configure port B for input
  i82c55a_write(
    REG_CTRL,
    CTRL_MODESET
      |CTRL_GROUPA_MODE0
      |CTRL_GROUPA_PORTA_OUT
      |CTRL_GROUPA_PORTC_UPPER_OUT
      |CTRL_GROUPB_MODE0
      |CTRL_GROUPB_PORTB_IN
      |CTRL_GROUPB_PORTC_LOWER_OUT);
  bus_delay();
}

/*
// do a simple counting animation
uint8_t anim = 0;
*/

void loop() {
  /*
  i82c55a_write(REG_PORTA, anim);
  anim++;
  delay(200);
  */
  // copy input read from DIP switch on port B to LEDs on port A
  uint8_t val = i82c55a_read(REG_PORTB);
  i82c55a_write(REG_PORTA, val);
}

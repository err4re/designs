# Structure of project

## GDS Hierarchy

Design can be a full Wafer, or just a chip. More commonly just a single chip.

1. Wafer
   1. Dose test chip
      1. Alignment marks
   2. Optical Chip
      1. Dicing reference
   3. **Chip**
      1. Frame
         1. PCB pads
         2. Corner marks
         3. Label
         4. Dicing reference
      2. Feedlines
         1. Microwave pads
         2. Microstrip line
         3. Spirals
         4. Quantum circuits
         5. Connectors
      3. Test structures
         1. Dose tests
            1. Junction tests
            2. Squid tests
            3. Test devices
            4. Single pass lines
         2. DC devices
            1. Resistors
            2. Junctions
               1. Two wire
               2. Four wire
            3. Squids
               1. Two wire
               2. Four wire

## Chip Notebook Workflow

### Quantum Circuits

1. Define junctions
2. Define squids
3. Define resistors
4. Define quantum circuits

### Resonators

1. Define spirals

### Chip

1. Generate frame
2. Generate feedline
   1. Add spirals
   2. Add quantum circuits
   3. Add connectors (to define coupling)
3. Generate test structures
   1. Generate DC test devices
      1. Generate DC pads
      2. Generate test resistors
      3. Generate two wire junction devices
      4. Generate two wire squid devices
      5. Generate four wire junction devices
      6. Generate four wire squid devices
4. Generate chip cheat sheet (for measurements)
5. Generate position lists (for ebeam exposure)
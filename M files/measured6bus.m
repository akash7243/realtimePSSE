function measured = measured6bus
% Base voltage in KV
measured.base_voltage = 230;
% Measured bus voltage in pu
measured.bus_voltage = [    238.4/230;
                            237.8/230;
                            250.7/230;
                            225.7/230;
                            225.2/230;
                            228.9/230];
% MVA base
measured.base_power = 100;                        
% Measured bus power in pu                        
measured.bus_power = [      113.1/100   20.2/100;
                            48.4/100    71.9/100;
                            55.1/100    90.6/100;                        
                            71.8/100    71.9/100;
                            72/100      67.7/100;
                            72.3/100    60.9/100];
% Measured branch flow in pu
%                               from   to         real       reactive
measured.transfer_power = [      1      2       31.5/100    -13.2/100;
                                 1      4       38.9/100     21.2/100;
                                 1      5       35.7/100     9.4/100;
                                 2      1      -34.9/100     9.7/100;
                                 2      4       32.8/100     38.3/100;
                                 2      5       17.4/100     22/100;    
                                 2      6       22.3/100     15/100;
                                 2      3       8.6/100     -11.9/100;
                                 3      2      -2.1/100      10.2/100; 
                                 3      5       17.7/100     23.9/100;
                                 3      6       43.3/100     58.3/100;
                                 4      1      -40.1/100    -14.3/100;
                                 4      2      -29.8/100    -44.3/100;
                                 4      5       0.7/100     -17.4/100;
                                 5      4      -2.1/100     -1.5/100;
                                 5      1      -36.6/100    -17.5/100;
                                 5      2      -11.7/100    -22.2/100;
                                 5      3      -25.1/100    -29.9/100;
                                 5      6      -2.1/100     -0.8/100;
                                 6      5       1/100        2.9/100;
                                 6      2      -19.6/100    -22.3/100;
                                 6      3      -46.8/100    -51.1/100];
                                 
                                 
                                 
                                 
                                 
                                 
                                 
                                 
                              
                                 
                                 
                                 
                                 
                                 
                                 
                                 
                                 
function [A,B,C,D] = getStateSpace()

A = [1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1];

 
 B = [0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      1 0 0 0 0 0 0 0 0;
      0 1 0 0 0 0 0 0 0;
      0 0 1 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 1 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0;
      0 0 0 0 0 1 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0;
      0 0 0 0 0 0 1 0 0;
      0 0 0 0 0 0 0 1 0;
      0 0 0 0 0 0 0 0 1];
  
 C = [1 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0;
      0 1 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0;
      0 0 1 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0;
      0 0 0 1 0 0 0 0 0 -1 0 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0 0 -1 0 0 0 0 0 0 0;
      0 0 0 0 0 1 0 0 0 0 0 -1 0 0 0 0 0 0;
      0 0 0 0 0 0 1 0 0 0 0 0 -1 0 0 0 0 0;
      0 0 0 0 0 0 0 1 0 0 0 0 0 -1 0 0 0 0;
      0 0 0 0 0 0 0 0 1 0 0 0 0 0 -1 0 0 0;
      0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 -1 0 0;
      0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 -1 0;
      0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 -1];
  
      
 C = [1 0 0 0 -1 0 0 0 0 0 0 0; 
      0 1 0 0 0 -1 0 0 0 0 0 0;
      0 0 1 0 0 0 -1 0 0 0 0 0;
      0 0 0 1 0 0 0 -1 0 0 0 0;
      0 0 0 0 1 0 0 0 -1 0 0 0;
      0 0 0 0 0 1 0 0 0 -1 0 0;
      0 0 0 0 0 0 1 0 0 0 -1 0;
      0 0 0 0 0 0 0 1 0 0 0 -1];
  
 D = [0 0 0 0 0 0 0 0 0 0 0 0]';
 
end

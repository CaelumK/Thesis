function [A,B,C,D] = twoDStateSpace()

A = [1 0 1 0 0 0 0 0 0 0 0 0;
     0 1 0 1 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 1 0 0 0 0 0;
     0 0 0 0 0 1 0 1 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 1 0;
     0 0 0 0 0 0 0 0 0 1 0 1;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1];
 
 B = [0 0 0 0 0 0;
      0 0 0 0 0 0;
      1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];
  
 C = [1 0 0 0 -1 0 0 0 0 0 0 0; 
      0 1 0 0 0 -1 0 0 0 0 0 0;
      0 0 1 0 0 0 -1 0 0 0 0 0;
      0 0 0 1 0 0 0 -1 0 0 0 0;
      0 0 0 0 1 0 0 0 -1 0 0 0;
      0 0 0 0 0 1 0 0 0 -1 0 0;
      0 0 0 0 0 0 1 0 0 0 -1 0;
      0 0 0 0 0 0 0 1 0 0 0 -1];
  
 D = zeros(8,6);
 
end
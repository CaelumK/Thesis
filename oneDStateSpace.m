function [A,B,C,D] = oneDStateSpace()

A = [1 1 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 1 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 1;
     0 0 0 0 0 1];
 
 B = [0 0 0;
      1 0 0;
      0 0 0;
      0 1 0;
      0 0 0;
      0 0 1];
  
 C = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      -1 0 1 0 0 0;
      0 0 0 1 0 0;
      0 0 -1 0 1 0;
      0 0 0 0 0 1];
  
 D = zeros(6,3); %same num rows as C, same num cols as B
 
end
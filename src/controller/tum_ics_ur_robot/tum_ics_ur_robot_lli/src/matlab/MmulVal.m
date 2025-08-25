function [ output_args ] = MmulVal( input_args )
%MMULVAL Summary of this function goes here
%   Detailed explanation goes here

syms a1 a2 a3 d1 d2 d3 c11 c12 c13 c21 c22 c23 c31 c32 c33 dq1 dq2 dq3 m11 m12 m13 m22 m23 m33 real


M=[m11 m12 m13;
      m12 m22 m23;
      m13 m23 m33]

Kd=diag([d1,d2,d3])

al=diag([a1,a2,a3])

C=[c11 c12 c13;
      c21 c22 c23;
      c31 c32 c33]
  
  Dq=[dq1;dq2;dq3]
  
  T1=(Kd*al*Dq)-(C*al*Dq)
  
  T2=(Kd-C)*al*Dq
  
  R=simplify(T1-T2)
  
  
  T1= (M\((Kd-C)*Dq))-al*Dq
  
  T2=((M\(Kd-C))-al)*Dq
  
  R=simplify(T1-T2)

end


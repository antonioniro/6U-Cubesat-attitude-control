function qprod=q_product(q1,q2)
% qprod=q1*q2
% It performs the quaternion multiplication 
% INPUT
% q1 and q2 are two quaternions with scalar-last convention,size [4x1]

qprod=[q1(4) q1(3) -q1(2) q1(1);
       -q1(3) q1(4) q1(1) q1(2);
       q1(2) -q1(1) q1(4) q1(3);
       -q1(1) -q1(2) -q1(3) q1(4)]*q2;
% 
% qprod=[q2(4).*q1(1:3)+q1(4).*q2(1:3)-cross(q1(1:3),q2(1:3));
%        q1(4)*q2(4)-dot(q1(1:3),q2(1:3))]

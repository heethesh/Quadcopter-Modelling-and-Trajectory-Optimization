syms r p y rdes pdes ydes J11 J22 J33 m g force;

A = [0,0,0,0,0,0,1,0,0,0,0,0;0,0,0,0,0,0,0,1,0,0,0,0;0,0,0, ...
     0,0,0,0,0,1,0,0,0;0,0,0,0,0,0,0,0,0,1,0,0;0,0,0,0,0,0,0,0,0,0,1,0; ...
     0,0,0,0,0,0,0,0,0,0,0,1;0,0,0,force.*m.^(-1).*((-1).*cos(y).*sin(p) ...
     .*sin(r)+cos(r).*sin(y)),force.*m.^(-1).*cos(p).*cos(r).*cos(y) ...
     ,force.*m.^(-1).*(cos(y).*sin(r)+(-1).*cos(r).*sin(p).*sin(y)), ...
     0,0,0,0,0,0;0,0,0,force.*m.^(-1).*((-1).*cos(r).*cos(y)+(-1).*sin( ...
     p).*sin(r).*sin(y)),force.*m.^(-1).*cos(p).*cos(r).*sin(y),force.* ...
     m.^(-1).*(cos(r).*cos(y).*sin(p)+sin(r).*sin(y)),0,0,0,0,0,0; ...
     0,0,0,(-1).*force.*m.^(-1).*cos(p).*sin(r),(-1).*force.*m.^(-1).*cos( ...
     r).*sin(p),0,0,0,0,0,0,0;0,0,0,J11.^(-1).*(J22+(-1).*J33).*((-1) ...
     .*pdes.^2.*cos(r).^2+ydes.^2.*cos(p).^2.*cos(r).^2+pdes.^2.*sin( ...
     r).^2+(-1).*ydes.^2.*cos(p).^2.*sin(r).^2+(-2).*pdes.*ydes.*cos( ...
     p).*sin(2.*r)),J11.^(-1).*(J22+(-1).*J33).*((-1).*pdes.*ydes.*cos( ...
     2.*r).*sin(p)+(-2).*ydes.^2.*cos(p).*cos(r).*sin(p).*sin(r)), ...
     0,0,0,0,0,J11.^(-1).*(J22+(-1).*J33).*(ydes.*cos(p).*cos(2.*r)+( ...
     -2).*pdes.*cos(r).*sin(r)),J11.^(-1).*(J22+(-1).*J33).*(pdes.*cos( ...
     p).*cos(2.*r)+2.*ydes.*cos(p).^2.*cos(r).*sin(r));0,0,0,J22.^( ...
     -1).*(J11+(-1).*J33).*(rdes+(-1).*ydes.*sin(p)).*(pdes.*cos(r)+ ...
     ydes.*cos(p).*sin(r)),J22.^(-1).*((J11+(-1).*J33).*ydes.*cos(r).* ...
     sin(p).*(rdes+(-1).*ydes.*sin(p))+(-1).*(J11+(-1).*J33).*ydes.*cos( ...
     p).*((-1).*ydes.*cos(p).*cos(r)+pdes.*sin(r))),0,0,0,0,J22.^(-1) ...
     .*(J11+(-1).*J33).*((-1).*ydes.*cos(p).*cos(r)+pdes.*sin(r)), ...
     J22.^(-1).*(J11+(-1).*J33).*(rdes+(-1).*ydes.*sin(p)).*sin(r), ...
     J22.^(-1).*((-1).*(J11+(-1).*J33).*cos(p).*cos(r).*(rdes+(-1).* ...
     ydes.*sin(p))+(-1).*(J11+(-1).*J33).*sin(p).*((-1).*ydes.*cos(p) ...
     .*cos(r)+pdes.*sin(r)));0,0,0,(J11+(-1).*J22).*J33.^(-1).*(rdes+( ...
     -1).*ydes.*sin(p)).*(ydes.*cos(p).*cos(r)+(-1).*pdes.*sin(r)), ...
     J33.^(-1).*((-1).*(J11+(-1).*J22).*ydes.*sin(p).*(rdes+(-1).*ydes.* ...
     sin(p)).*sin(r)+(-1).*(J11+(-1).*J22).*ydes.*cos(p).*(pdes.*cos( ...
     r)+ydes.*cos(p).*sin(r))),0,0,0,0,(J11+(-1).*J22).*J33.^(-1).*( ...
     pdes.*cos(r)+ydes.*cos(p).*sin(r)),(J11+(-1).*J22).*J33.^(-1).* ...
     cos(r).*(rdes+(-1).*ydes.*sin(p)),J33.^(-1).*((J11+(-1).*J22).* ...
     cos(p).*(rdes+(-1).*ydes.*sin(p)).*sin(r)+(-1).*(J11+(-1).*J22).* ...
     sin(p).*(pdes.*cos(r)+ydes.*cos(p).*sin(r)))];

B = [0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0;m.^( ...
     -1).*(cos(r).*cos(y).*sin(p)+sin(r).*sin(y)),0,0,0;m.^(-1).*( ...
     (-1).*cos(y).*sin(r)+cos(r).*sin(p).*sin(y)),0,0,0;m.^(-1).* ...
     cos(p).*cos(r),0,0,0;0,J11.^(-1),0,0;0,0,J22.^(-1),0;0,0,0, ...
     J33.^(-1)];

A = simplify(A);
B = simplify(B);

matlabFunction(A, 'File', 'state_space_A', 'vars', {r p y rdes pdes ydes J11 J22 J33 m g force});
matlabFunction(B, 'File', 'state_space_B', 'vars', {r p y rdes pdes ydes J11 J22 J33 m g force});

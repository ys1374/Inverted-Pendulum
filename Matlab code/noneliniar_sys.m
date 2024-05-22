function dx = noneliniar_sys(x,m,J,L_R,L_P,g,K,Ra,b0,Ea)

Sx = sin(x(1));
Cx = cos(x(1));


dx(1,1) = x(2);
dx(2,1) =((J+((L_R^2)*m))/J)*((g*Sx)/(L_P*(Cx^2)))-(Sx/Cx)*x(2)-((K*L_R)/(Ra*J*L_P*Cx))*Ea+((L_R*(Ra*b0+(K.^2)))/(Ra*J*L_P*Cx))*x(4);
dx(3,1) = x(4);
dx(4,1) = (K/(Ra*J))*Ea-((Ra*b0+(K^2))/(Ra*J))*x(4)-((g*m*L_R*Sx)/(J*Cx));


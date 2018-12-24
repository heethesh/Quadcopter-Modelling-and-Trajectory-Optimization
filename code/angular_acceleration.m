function dww_des = angular_acceleration(exx_,eyy_,exxx_,eyyy_,xx_des_,yy_des_,xxx_des_,yyy_des_,xxxx_des_,yyyy_des_,psid_,psidd_,psiddd_,g,t)
%ANGULAR_ACCELERATION
%    DWW_DES = ANGULAR_ACCELERATION(EXX_,EYY_,EXXX_,EYYY_,XX_DES_,YY_DES_,XXX_DES_,YYY_DES_,XXXX_DES_,YYYY_DES_,PSID_,PSIDD_,PSIDDD_,G,T)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    26-Oct-2018 23:54:04

t2 = 1.0./g;
t3 = cos(psid_);
t4 = eyy_+yy_des_;
t5 = sin(psid_);
t6 = exx_+xx_des_;
t7 = psiddd_.*t6;
t8 = exxx_+xxx_des_;
t9 = psidd_.*t8;
t10 = t7+t9-yyyy_des_;
t11 = psiddd_.*t4;
t12 = eyyy_+yyy_des_;
t13 = psidd_.*t12;
t14 = t11+t13+xxxx_des_;
t15 = yyy_des_-psidd_.*t6;
t16 = psidd_.*t4;
t17 = t16+xxx_des_;
dww_des = [t2.*t3.*t10+t2.*t5.*t14+psidd_.*t2.*t3.*t17+psidd_.*t2.*t5.*t15;-t2.*t5.*t10+t2.*t3.*t14+psidd_.*t2.*t3.*t15-psidd_.*t2.*t5.*t17];

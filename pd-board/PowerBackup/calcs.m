Vout = 5;
Vin = 9;
Vin_max = 12;
fsw=600e3;
ts = 1/fsw;

Cout = 51.1e-6

Iout_max = 9;

t_ON_estimated = Vout/(Vin*600e3);
t_OFF_min = 300e-9;

D_max = 1 - t_OFF_min/ts

L = Vout*(Vin_max-Vout)/(Vin_max*fsw*0.2*Iout_max)
L_chosen = 3.3e-6;

d_Ipp = Vout*(Vin_max-Vout)/(Vin_max*fsw*L_chosen)

IL_pk = Iout_max + 0.5*d_Ipp

IL_rms = sqrt(Iout_max^2+d_Ipp^2/12)
PL_loss = IL_rms^2*9.42e-3

d_Vout_pp = sqrt((d_Ipp/(8*fsw*Cout))^2+(d_Ipp*100e-3)^2)

d_Vfb_pp = 0.01**d_Ipp


Kdiv = d_Vfb_pp/Vin *fsw/(D_max*(1-D_max))

Rinj = 157.5/30*(1/Kdiv-1)
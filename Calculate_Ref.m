function [Ref] = Calculate_Ref(KRob, Rated, Test_parameters)


%% Primary reference values
Ref.w_hss = Rated.w_hss; % From Manufacturer's Datasheet (MD)

Ref.tau_hss = Rated.tau_hss ; % N m % From Manufacturer's Datasheet (MD)
Ref.i = Rated.i; % From Manufacturer's Datasheet (MD)

Ref.alpha = 1;
Ref.length = KRob.rdrum;

%% Secondary reference values
Ref.w =             Ref.w_hss / KRob.rgt;
Ref.t =             1/Ref.w;							
Ref.tau =           Ref.tau_hss * KRob.rgt;
Ref.tension =   Ref.tau / Ref.length;
Ref.vel =           Ref.w * Ref.length;
Ref.alpha_hss = Ref.alpha * KRob.rgt;
Ref.PID_gain =  Ref.i / KRob.rgt;
Ref.kphi =          Ref.tau_hss/Ref.i;
Ref.B_hss =      Ref.tau_hss / Ref.w_hss;
Ref.B  =            Ref.tau / Ref.w;
Ref.J_hss =     Ref.tau_hss / Ref.w_hss *Ref.t;
Ref.J =             Ref.tau / Ref.w * Ref.t;
Ref.m =             Ref.tau / Ref.w^2 / Ref.length^2;
Ref.kel_esp =    Ref.tau / Ref.length;
Ref.acel =          Ref.tension / Ref.m;
Ref.acel_ang =  1/ Ref.w^2;

if ~Test_parameters.normalize % All ref = 1 if no normalization
    S = struct(Ref);      % convierte a struct
    f = fieldnames(S);    
    
    for i = 1:numel(f)
        Ref.(f{i}) = 1;
    end
end

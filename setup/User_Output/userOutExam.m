function [UserSimOut] = userOutFu(SimOut)

  UserSimOut.Env              = SimOut.Env;
  UserSimOut.a                = SimOut.EOM;
  UserSimOut.FM                 = SimOut.FM;
  UserSimOut.Traj_Iner          = SimOut.Traj_Iner;
  UserSimOut.RefInputs          = SimOut.RefInputs;
  UserSimOut.Misc.EngCmd        = SimOut.EngBus;
  UserSimOut.EOM_Sensed         = SimOut.EOM_Sensed;
  UserSimOut.Env_Sensed         = SimOut.Env_Sensed;
end


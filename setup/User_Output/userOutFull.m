function [UserSimOut] = userOutFull(SimOut)

  UserSimOut.Env              = SimOut.Env;
  UserSimOut.EOM                = SimOut.EOM;
  UserSimOut.FM                 = SimOut.FM;
  UserSimOut.Traj_Iner          = SimOut.Traj_Iner;
  UserSimOut.RefInputs          = SimOut.RefInputs;
  UserSimOut.Misc.EngCmd        = SimOut.EngBus;
  UserSimOut.EOM_Sensed         = SimOut.EOM_Sensed;
  UserSimOut.Env_Sensed         = SimOut.Env_Sensed;
  UserSimOut.Env_Sensed         = SimOut.Env_Sensed;
  UserSimOut.Bball_Traj_Iner    = SimOut.Bball_Traj_Iner;
end


function SimIn = setupEnvironment(SimIn,userStruct)
SimIn.Environment.Atmos = setupAtmosphere(SimIn);
SimIn.Environment.Earth = setupEarth(SimIn);
SimIn=setupWinds(SimIn,userStruct);
end
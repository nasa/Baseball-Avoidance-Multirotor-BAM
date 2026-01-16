% define EOM variant conditions
dataOut_m = enumeration('DataOutEnum');
dataOut_len = length(dataOut_m);
for i=1:dataOut_len
  cond_expr = sprintf('SimIn.variants.dataOutType == DataOutEnum.%s', dataOut_m(i));
  switch dataOut_m(i)
      case DataOutEnum.NORMAL
          GVS_DATAOUT_TYPE_NORMAL = Simulink.Variant(cond_expr);
      case DataOutEnum.CODEGEN
          GVS_DATAOUT_TYPE_CODEGEN = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear dataOut_m dataOut_len i cond_expr;
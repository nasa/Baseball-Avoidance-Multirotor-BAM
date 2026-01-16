% define Publisher variant conditions
sub_m = enumeration('SubEnum');
sub_len = length(sub_m);
for i=1:sub_len
  cond_expr = sprintf('SimIn.variants.subType == SubEnum.%s', sub_m(i));
  switch sub_m(i)
      case PubEnum.NONE
          SUB_TYPE_NONE = Simulink.Variant(cond_expr);
      case PubEnum.ROS2
          SUB_TYPE_PHASE = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear sub_m sub_len i cond_expr;
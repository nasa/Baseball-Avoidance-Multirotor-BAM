% define autocode RT pacing variant conditions
sub_m = enumeration('PaceEnum');
sub_len = length(sub_m);
for i=1:sub_len
  cond_expr = sprintf('SimIn.variants.rt_pacing == PaceEnum.%s', sub_m(i));
  switch sub_m(i)
      case PaceEnum.RT_NONE
          RT_TYPE_NONE = Simulink.Variant(cond_expr);
      case PaceEnum.RT_PACE
          RT_TYPE_PACE = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear sub_m sub_len i cond_expr;
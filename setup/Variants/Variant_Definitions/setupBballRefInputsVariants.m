% define refinput variant conditions
refin_m = enumeration('Bball_RefInputEnum');
refin_len = length(refin_m);
for i=1:refin_len
  cond_expr = sprintf('SimIn.variants.refInputTypeBball == Bball_RefInputEnum.%s', refin_m(i));
  switch refin_m(i)
      case Bball_RefInputEnum.NONE_BBALL
      GVS_BBALL_REFINPUT_TYPE_NONE = Simulink.Variant(cond_expr);
    case Bball_RefInputEnum.BEZIER_BBALL
      GVS_BBALL_REFINPUT_TYPE_BEZIER = Simulink.Variant(cond_expr);
    otherwise
  end
end

clear refin_m refin_len i cond_expr;
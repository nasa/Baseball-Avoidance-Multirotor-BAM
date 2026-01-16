% define Publisher variant conditions
pub_m = enumeration('PubEnum');
pub_len = length(pub_m);
for i=1:pub_len
  cond_expr = sprintf('SimIn.variants.pubType == PubEnum.%s', pub_m(i));
  switch pub_m(i)
      case PubEnum.NONE
          PUB_TYPE_NONE = Simulink.Variant(cond_expr);
      case PubEnum.ROS2
          PUB_TYPE_ROS2 = Simulink.Variant(cond_expr);
      case PubEnum.ROS2_NONLIB
          PUB_TYPE_ROS2_NONLIB = Simulink.Variant(cond_expr);
      otherwise
  end
end

clear pub_m pub_len i cond_expr;
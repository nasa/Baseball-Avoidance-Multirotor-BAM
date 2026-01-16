classdef Bball_RefInputEnum < Simulink.IntEnumType
  enumeration
    NONE_BBALL(1) % Note autocoding didn't like this being the same as RefInputEnum, so appending suffix "_BBALL"
    BEZIER_BBALL(2) % Note autocoding didn't like this being the same as RefInputEnum, so appending suffix "_BBALL"
  end
end

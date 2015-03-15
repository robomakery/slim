FILE(REMOVE_RECURSE
  "CMakeFiles/test.dir/test.cpp.o"
  "../bin/test.pdb"
  "../bin/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

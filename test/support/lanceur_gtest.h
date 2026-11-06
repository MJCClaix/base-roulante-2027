// Point d'entrée commun des suites googletest (à inclure en fin de fichier de test).
#pragma once

#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

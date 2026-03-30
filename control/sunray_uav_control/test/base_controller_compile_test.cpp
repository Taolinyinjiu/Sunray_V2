#include <gtest/gtest.h>

#include "controller/base_controller/base_controller.hpp"

namespace {

TEST(BaseControllerCompileTest, HeaderIsAvailableToConsumers) {
  OriginalControl *controller = nullptr;

  EXPECT_EQ(controller, nullptr);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

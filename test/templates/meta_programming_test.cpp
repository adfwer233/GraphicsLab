// #include "language/meta_programming/static_for_loop.hpp"
// #include "language/meta_programming/name_of.hpp"
//
// #include "gtest/gtest.h"
//
// struct CustomName {
//
// };
//
// enum CustomEnum {
//     testname1,
//     testname2
// };
//
// #if defined(_MSC_VER)
//
// TEST(MetaProgrammingTest, NameOf) {
//     auto tmp = MetaProgramming::type_name_of<CustomName>();
//     std::cout << tmp.str << std::endl;
//     std::cout << MetaProgramming::type_name_of<CustomEnum, testname1>().str << std::endl;
//     std::cout << std::boolalpha;
//     // std::cout << MetaProgramming::is_member_of_enum<CustomEnum, static_cast<CustomEnum>(100)>() << std::endl;
//     std::cout << MetaProgramming::is_member_of_enum<CustomEnum, testname1>() << std::endl;
//     for (auto item: MetaProgramming::all_name_of_enum<CustomEnum>())
//         std::cout << item << ' ';
//     std::cout << std::endl;
// }

// #endif
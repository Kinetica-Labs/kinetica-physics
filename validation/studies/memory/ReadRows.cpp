// Отдельная единица трансляции: каждый вариант обязан действительно передать построенный массив.
#include <joints/Bond.h>
#include <cstddef>

double readRows(const kp::Bond* rows, std::size_t size) {
    double total = 0;
    for (std::size_t i = 0; i < size; ++i) {
        const kp::Bond& r = rows[i];
        total += r.linear.dot({1,2,3}) + r.angularA.dot({2,3,1}) + r.angularB.dot({3,1,2}) + r.error;
    }
    return total;
}

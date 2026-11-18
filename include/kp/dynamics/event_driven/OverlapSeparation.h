#pragma once
// Раздвигание тел, вошедших друг в друга, и возврат разошедшихся сочленений: сдвигом поз, без
// изменения скоростей.

#include <kp/collision/ContactPair.h>
#include <kp/joints/JointSet.h>

#include <vector>

namespace kp {

class OverlapSeparation {
public:
    // up — единичный вектор против силы тяжести.
    OverlapSeparation(std::vector<Body>& bodies, const JointSet& joints, Vector3 up) : bodies_(bodies), joints_(joints), up_(up) {}
    // Возвращает, были ли перекрытия.
    bool separate(const std::vector<ContactPair>& touching);

private:
    std::vector<Body>& bodies_;
    const JointSet& joints_;
    Vector3 up_;
};

} // namespace kp

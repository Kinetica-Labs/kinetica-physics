#pragma once
// Раздвигание тел, вошедших друг в друга, и возврат разошедшихся сочленений: сдвигом поз, без
// изменения скоростей.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <joints/JointSet.h>

#include <vector>

namespace kp {

class OverlapSeparation {
public:
    // up — единичный вектор против силы тяжести.
    OverlapSeparation(std::vector<Body>& bodies, const JointSet& joints, Vector3 up) : bodies_(bodies), joints_(joints), up_(up) {}
    // Возвращает, были ли перекрытия.
    bool separate(const std::vector<ContactPair>& touching);

private:
    // Сдвиг поз на shifts без изменения импульсов меняет момент импульса системы на Σ Δr × p. Свободной
    // группе тел (связанной парами pairs и сочленениями, без неподвижных тел) он возвращается твёрдой
    // поправкой скоростей — поворотом b вокруг её центра масс, I b = −Σ Δr × p, I — тензор инерции
    // группы: импульс и момент импульса группы сохраняются точно
    // (proofs/separation_keeps_momenta.smt2). Разведение увеличивает I, и энергия
    // L² / 2I убывает — как у фигуриста, разводящего руки. Группа с неподвижным телом не трогается: её
    // импульсы уходят в опору.
    void keepMomenta(const std::vector<ContactPair>& pairs, const StepArray<Vector3>& shifts);
    // Корень группы каждого тела: тела, связанные парами с точками контакта и сочленениями; корень —
    // младший номер. Группа с неподвижным телом помечена корнем −1.
    StepArray<int> freeGroups(const std::vector<ContactPair>& pairs) const;
    // Глубина пар по истинной нормали там, где точки контакта её занижают; ложь — перекрытий нет.
    bool deepen(std::vector<ContactPair>& pushed) const;

    std::vector<Body>& bodies_;
    const JointSet& joints_;
    Vector3 up_;
};

} // namespace kp

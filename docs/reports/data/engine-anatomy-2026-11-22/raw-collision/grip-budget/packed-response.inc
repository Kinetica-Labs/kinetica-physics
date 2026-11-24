        // Один столбец — отклик от единичного импульса. Четыре независимых столбца помещаются в YMM.
        const auto& columns = onBody[index];
        for (size_t first = 0; first < columns.size(); first += 4) {
            const size_t count = std::min(size_t(4), columns.size() - first);
            alignas(32) double linear[3][4]{}, angular[3][4]{};
            for (size_t lane = 0; lane < count; ++lane) {
                const Line& source = lines[columns[first + lane]];
                const bool atB = source.b == int(index);
                const Real sign = atB ? Real(1) : Real(-1);
                const Vector3 v = (sign * body.mass.inverseMass) * source.linear;
                const Vector3 w = sign * inverseInertiaTimes(body, atB ? source.angularB : source.angularA);
                linear[0][lane] = v.x; linear[1][lane] = v.y; linear[2][lane] = v.z;
                angular[0][lane] = w.x; angular[1][lane] = w.y; angular[2][lane] = w.z;
            }
            const auto dotFour = [](Vector3 row, const double (&v)[3][4]) {
                const __m256d x = _mm256_mul_pd(_mm256_set1_pd(row.x), _mm256_load_pd(v[0]));
                const __m256d y = _mm256_mul_pd(_mm256_set1_pd(row.y), _mm256_load_pd(v[1]));
                const __m256d z = _mm256_mul_pd(_mm256_set1_pd(row.z), _mm256_load_pd(v[2]));
                return _mm256_add_pd(_mm256_add_pd(x, y), z);
            };
            for (size_t i : columns) {
                const Line& target = lines[i];
                const bool atB = target.b == int(index);
                const __m256d values = _mm256_add_pd(dotFour(target.linear, linear),
                    dotFour(atB ? target.angularB : target.angularA, angular));
                alignas(32) double coefficient[4];
                _mm256_store_pd(coefficient, values);
                for (size_t lane = 0; lane < count; ++lane)
                    matrix[i].push_back({columns[first + lane], atB ? coefficient[lane] : -coefficient[lane]});
            }
        }

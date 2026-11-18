#pragma once
// Просмотр отладочной геометрии в браузере: все кадры пишутся в один файл HTML без зависимостей.
//
// Движок отдаёт только точки и отрезки (debug/DebugGeometry.h). Здесь они проецируются на экран
// и раскрашиваются. Любой другой рендер — OpenGL, Vulkan — подключается так же: получает тот же
// список и рисует его по-своему.

#include <kp/debug/DebugGeometry.h>

#include <cstdio>
#include <string>

class HtmlViewer {
public:
    // centre — точка сцены в середине экрана; width — сколько метров сцены помещается по ширине.
    HtmlViewer(kp::Vector3 centre, double width) : centre_(centre), scale_(kWidth / width) {}

    void frame(const kp::DebugGeometry& geometry) {
        frames_ += "{l:[";
        for (const kp::DebugLine& line : geometry.lines) {
            const Pixel a = project(line.from), b = project(line.to);
            frames_ += text("[%.1f,%.1f,%.1f,%.1f,%d],", a.x, a.y, b.x, b.y, int(line.kind));
        }
        frames_ += "],p:[";
        for (const kp::DebugPoint& point : geometry.points) {
            const Pixel p = project(point.position);
            frames_ += text("[%.1f,%.1f,%d],", p.x, p.y, int(point.kind));
        }
        frames_ += "]},\n";
    }

    // Записывает страницу: ползунок листает кадры, щелчок по картинке — пуск и пауза.
    void save(const std::string& path) const {
        std::FILE* file = std::fopen(path.c_str(), "w");
        if (!file) return;
        std::fprintf(file, kPage, kWidth, kHeight, frames_.c_str(), kWidth, kHeight);
        std::fclose(file);
        std::printf("картинка: %s\n", path.c_str());
    }

private:
    static constexpr int kWidth = 800, kHeight = 600;
    struct Pixel {
        double x, y;
    };

    // Камера смотрит сбоку и немного сверху; ось Y мира направлена вверх экрана.
    Pixel project(kp::Vector3 point) const {
        const kp::Vector3 d = point - centre_;
        const double right = 0.8 * d.x - 0.6 * d.z, depth = 0.6 * d.x + 0.8 * d.z;
        return {kWidth / 2 + scale_ * right, kHeight / 2 - scale_ * (d.y - 0.3 * depth)};
    }

    template <class... Values> static std::string text(const char* format, Values... values) {
        char buffer[96];
        std::snprintf(buffer, sizeof buffer, format, values...);
        return buffer;
    }

    // Цвета по DebugKind: тело, неподвижное тело, точка контакта, нормаль контакта, ближайшие точки.
    static constexpr const char* kPage = R"(<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>kinetica-physics</title><body style="margin:0;background:#111;color:#ccc;font:14px sans-serif">
<canvas id="c" width="%d" height="%d" style="width:100%%;max-width:800px"></canvas><br>
<input id="s" type="range" min="0" value="0" style="width:90%%"> <span id="n"></span>
<script>
const F=[%s];
const colour=["#9cf","#777","#f44","#fc3","#4f8","#96c"];
const c=document.getElementById("c").getContext("2d"),s=document.getElementById("s"),n=document.getElementById("n");
s.max=F.length-1;
function draw(){const f=F[+s.value];c.clearRect(0,0,%d,%d);
 for(const l of f.l){c.strokeStyle=colour[l[4]];c.beginPath();c.moveTo(l[0],l[1]);c.lineTo(l[2],l[3]);c.stroke();}
 for(const p of f.p){c.fillStyle=colour[p[2]];c.beginPath();c.arc(p[0],p[1],4,0,7);c.fill();}
 n.textContent="кадр "+s.value+" из "+(F.length-1);}
function next(){s.value=(+s.value+1)%%F.length;draw();}
let timer=F.length>1?setInterval(next,33):0;
c.canvas.onclick=()=>{if(timer){clearInterval(timer);timer=0;}else timer=setInterval(next,33);};
s.oninput=draw;draw();
</script>)";

    kp::Vector3 centre_;
    double scale_;
    std::string frames_;
};

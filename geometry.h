#include <iostream>
#include <cmath>
#include <vector>

const double PI = M_PI;
const double epsilon = 0.000000001;

struct Point{
    double x;
    double y;

    Point(double X, double Y){
        x = X;
        y = Y;
    }

    Point() = default;

    bool operator==(const Point& b) const{
        return fabs(b.x - x) - epsilon < 0 && fabs(b.y - y) - epsilon < 0;
    }

    bool operator!=(const Point& b) const{
        return !operator==(b);
    }

    bool operator=(const Point& point){
        x = point.x;
        y = point.y;
    }

    Point (const Point& point) {
        x = point.x;
        y = point.y;
    }

    void rotate(Point center, double angle){
        double X = center.x + (x - center.x) * cos(angle * PI / 180) - (y - center.y) * sin(angle * PI / 180);
        double Y = center.y + (x - center.x) * sin(angle * PI / 180) + (y - center.y) * cos(angle * PI / 180);
        x = X;
        y = Y;
    }

    void reflex(Point center){
        x = x + 2 * (center.x - x);
        y = y + 2 * (center.y - y);
    }

    void scale(Point center, double coefficient){
        x = center.x + coefficient * (x - center.x);
        y = center.y + coefficient * (y - center.y);
    }

    double distance(Point p) const{
        return pow(pow(p.x - x, 2) + pow(p.y - y, 2), 0.5);
    }

    double Vector_Mult(Point b, Point c, Point d) const {
        return (b.x - x) * (d.y - c.y) - (b.y - y) * (d.x - c.x);
    }

    Point center(Point another) const{
        return Point((x + another.x) / 2, (y + another.y) / 2);
    }

    double angle(Point A, Point B) const{
        double b = A.distance(B);
        double a = distance(B);
        double c = distance(A);
        return acos((pow(b , 2) + pow(c, 2) - pow(a, 2))/(2 * b * c));
    }
};

class Line{
public:

    std::vector<double> for_tests_get_coeff(){
        std::vector<double> v(0);
        v.push_back(a);
        v.push_back(b);
        v.push_back(c);
        return v;
    }

    Line(Point p, Point q){
        if (fabs(q.y - p.y) < epsilon) {
            a = 0;
            b = 1;
            c = -p.y;
        }
        else {
            a = 1;
            b = (p.x - q.x)/(q.y - p.y);
            c = - b * p.y - p.x;
        }
    }

    Line(Point p, double k){
        a = k;
        b = -1;
        c =  p.y - a * p.x;
    }

    Line(double m, double k){
        a = k;
        c = m;
        b = -1;
    }

    Line() = default;

    Line(double A, double B, double C){
        a = A;
        b = B;
        c = C;
    }

    bool operator==(const Line& line) const{
        if (a != 0){
            if (line.a == 0){
                return false;
            }
            return (fabs(line.b / line.a - b / a) < epsilon && fabs(line.c / line.a - c / a) < epsilon);
        }
        if (line.b == 0){
            return false;
        }
        return (fabs(line.a / line.b - a / b) < epsilon && fabs(line.c / line.b - c / b) < epsilon);
    }

    bool operator!=(const Line& line) const{
        return !operator==(line);
    }

    bool operator=(const Line& line){
        a = line.a;
        b = line.b;
        c = line.c;
    }

    Line (const Line& line) {
        a = line.a;
        b = line.b;
        c = line.c;
    }

    bool parall(Line another) const{
        if (fabs(another.a * b - another.b * a) < epsilon){
            return true;
        }
        return false;
    }

    Point intersection(Line another) const{
        std::vector<double> coeff = another.for_tests_get_coeff();
        double x, y;
        if (a == 0) {
            y = -c/b;
            x = ( - another.c - y * another.b) / another.a;
        }
        else if (another.b * a != another.a * b) {
            y = - (another.c * a - c * another.a)/(another.b * a - another.a * b);
            x = (- c - y * b) / a;
        }
        return Point(x, y);
    }

    bool PointOnLine(Point p) const{
        return fabs(p.x * a + p.y * b + c) < epsilon;
    }

    bool PointOnSegment(Point point, Point first, Point second) const{
        if (PointOnLine(point) && PointOnLine(first) && PointOnLine(second)){
            if (((second.x - point.x) * (point.x - first.x) >= 0) && ((second.y - point.y) * (point.y - first.y) >= 0)){
                return true;
            }
        }
        return false;
    }

    void rotate(Point center, double angle){
        std::pair<Point, Point> points = find_two_points();
        points.first.rotate(center, angle);
        points.second.rotate(center, angle);
        set_coeff(points.first, points.second);
    }

    void reflex(Point center){
        std::pair<Point, Point> points = find_two_points();
        points.first.reflex(center);
        points.second.reflex(center);
        set_coeff(points.first, points.second);
    }

    void reflex(Line axis){
        std::pair<Point, Point> points = find_two_points();
        std::vector<double> coeff = axis.for_tests_get_coeff();
        points.first.reflex(axis.perp_base(points.first));
        points.second.reflex(axis.perp_base(points.second));
        set_coeff(points.first, points.second);
    }

    Point perp_base(Point p){
        Line perp = perpendicular(p);
        Point q = intersection(perp);
        return q;
    }

    Line perpendicular(Point p) const{
        return Line(-b, a, -p.y * a + b * p.x);
    }

   void scale(Point center, double coefficient){
       std::pair<Point, Point> points = find_two_points();
       points.first.scale(center, coefficient);
       points.second.scale(center, coefficient);
       set_coeff(points.first, points.second);
    }
private:
    double a; // прямая задаётся уравнением ax + by + c = 0
    double b;
    double c;

    std::pair<Point, Point> find_two_points(){
        if (b == 0){
            return std::make_pair(Point(-c/a, 0), Point(-c/a, 1));
        }
        return std::make_pair(Point(0, -c/b), Point(1, (- c - a)/b));
    }

    void set_coeff(Point p, Point q){
        if (fabs(q.y - p.y) < epsilon) {
            a = 0;
            b = 1;
            c = -p.y;
        }
        else {
            a = 1;
            b = (p.x - q.x)/(q.y - p.y);
            c = - b * p.y - p.x;
        }
    }
};

class Shape{
public:
    virtual double perimeter() const = 0;
    virtual double area() const = 0;
    virtual bool operator==(const Shape& another) const = 0;
    virtual bool operator!=(const Shape& another) const = 0;
    virtual bool isCongruentTo(const Shape& another) const = 0;
    virtual bool isSimilarTo(const Shape& another) const = 0;
    virtual bool containsPoint(Point point) const = 0;
    virtual void rotate(Point center, double angle) = 0;
    virtual void reflex(Point center) = 0;
    virtual void reflex(Line axis) = 0;
    virtual void scale(Point center, double coefficient) = 0;
    virtual ~Shape() = default;
};

class Polygon: public Shape{
public:
    explicit Polygon(const std::vector<Point>& vertices){
        Vertices = vertices;
    }

    template <class First, class ... InQueue>
    explicit Polygon(const First& first, const InQueue& ... queue) {
        Add_Vertices(first, queue ...);
    }

    Polygon() = default;

    void Add_Vertices(){};

    template <class First, class ... InQueue>
    void Add_Vertices(const First& first, const InQueue& ... queue) {
        Vertices.push_back(first);
        Add_Vertices(queue ...);
    }

    int verticesCount() const{
        return Vertices.size();
    }

    std::vector<Point> getVertices() const{
        return Vertices;
    }

    bool isConvex(){
        double Is_left = Vertices[Vertices.size() - 1].Vector_Mult(Vertices[0], Vertices[0], Vertices[1]);
        for (size_t i = 0; i < Vertices.size() - 2; i++) {
            if (Is_left * Vertices[i].Vector_Mult(Vertices[i + 1], Vertices[i + 1], Vertices[i + 2]) < 0){
                return false;
            }
        }
        return Is_left * Vertices[Vertices.size() - 2].Vector_Mult(Vertices[Vertices.size() - 1], Vertices[Vertices.size() - 1], Vertices[0]) >= 0;
    }

    double perimeter() const override {
        double perimeter = 0;
        for (size_t i = 0; i < Vertices.size() - 1; i++) {
            perimeter += Vertices[i].distance(Vertices[i + 1]);
        }
        perimeter += Vertices[0].distance(Vertices[Vertices.size() - 1]);
        return perimeter;
    };

    double area() const override {
        double left_summ = 0, right_summ = 0;
        for (size_t i = 0; i < Vertices.size() - 1; i++) {
            left_summ += Vertices[i].x * Vertices[i + 1].y;
            right_summ += Vertices[i + 1].x * Vertices[i].y;
        }
        left_summ += Vertices[Vertices.size() - 1].x * Vertices[0].y;
        right_summ += Vertices[Vertices.size() - 1].y * Vertices[0].x;
        return (left_summ - right_summ) / 2;
    };

    std::vector<double> GetAngles() const{
        std::vector<double> angles(0);
        angles.push_back(Vertices[Vertices.size() - 1].angle(Vertices[0], Vertices[1]));
        for (size_t i = 0; i < Vertices.size() - 2; i++){
            angles.push_back(Vertices[i].angle(Vertices[i + 1], Vertices[i + 2]));
        }
        angles.push_back(Vertices[Vertices.size() - 2].angle(Vertices[Vertices.size() - 1], Vertices[0]));
        return angles;
    }

    std::vector<double> GetSidesLen() const{
        std::vector<double> sides(0);
        for (size_t i = 0; i < Vertices.size() - 1; i++){
            sides.push_back(Vertices[i].distance(Vertices[i + 1]));
        }
        sides.push_back(Vertices[0].distance(Vertices[Vertices.size() - 1]));
        return sides;
    };

    bool Is_equal(std::vector<double> first, std::vector<double> second, int shift) const{
        bool is_direction = true;
        for (size_t i = 0; i < first.size(); i++){
            if (fabs(first[i] - second[(i + shift) % first.size()]) > epsilon){
                is_direction = false;
                break;
            }
        }
        if (!is_direction){
            for (size_t i = 0; i < first.size(); i++){
                if (fabs(first[i] - second[(first.size() - i + shift) % first.size()]) > epsilon){
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<int> Is_equal_with_shift(std::vector<double> first, std::vector<double> second) const{
        std::vector<int> shifts(0);
        if (first.size() != second.size()){
            return shifts;
        }
        for (size_t i = 0; i < first.size(); i++){
            if (fabs(first[0] - second[i]) < epsilon){
                if (Is_equal(first, second, i)){
                    shifts.push_back(i);
                }
            }
        }
        return shifts;
    }

    bool operator==(const Shape& another) const override{
        const Shape* another_pointer = &another;
        auto polygon = dynamic_cast<const Polygon*>(another_pointer);
        if (polygon){
            if (polygon->verticesCount() != verticesCount()){
                return false;
            }
            int begin_numb = -1;
            int direction = 0;
            for (size_t i = 0; i < Vertices.size(); i++){
                if (Vertices[i] == polygon->Vertices[0]){
                    begin_numb = i;
                    break;
                }
            }
            if (begin_numb != -1){
                if ((begin_numb != 0 && Vertices[begin_numb - 1] == polygon->Vertices[polygon->verticesCount() - 1]) || (begin_numb == 0 && Vertices[verticesCount() - 1] == polygon->Vertices[polygon->verticesCount() - 1])){
                    direction = -1;
                }
                else {
                    direction = 1;
                }
            }
            else {
                return false;
            }
            for (int i = 0; i < verticesCount(); i++){
                if (polygon->Vertices[i] != Vertices[(verticesCount() + begin_numb - i * direction) % verticesCount()]){
                    return false;
                }
            }
            return true;
        }
        return false;
    };

    bool operator!=(const Shape& another) const override{
        return !operator==(another);
    };

    bool isCongruentTo(const Shape& another) const override{
        const Shape* another_pointer = &another;
        auto polygon = dynamic_cast<const Polygon*>(another_pointer);
        if (polygon){
            std::vector<double> angles = GetAngles();
            std::vector<double> another_angles = polygon->GetAngles();
            std::vector<double> sides = GetSidesLen();
            std::vector<double> another_sides = polygon->GetSidesLen();
            std::vector<int> shifts = Is_equal_with_shift(angles, another_angles);
            for (size_t i = 0; i < shifts.size(); i++){
                if (Is_equal(sides, another_sides, shifts[i]) or Is_equal(sides, another_sides, shifts[i] - 1)){
                    return true;
                }
            }
        }
        return false;
    };

    bool isSimilarTo(const Shape& another) const override{
        const Shape* another_pointer = &another;
        auto polygon = dynamic_cast<const Polygon*>(another_pointer);
        if (polygon){
            std::vector<double> angles = GetAngles();
            std::vector<double> another_angles = polygon->GetAngles();
            std::vector<int> shifts = Is_equal_with_shift(angles, another_angles);
            if (!shifts.empty()) {
                return true;
            }
        }
        return false;
    };

    bool containsPoint(Point point) const override{
        Line line = Line(point, Point(point.x + 1, point.y));
        int Inters_Numb = 0;
        for (size_t i = 0; i < Vertices.size(); i++){
            Line side = Line(Vertices[i], Vertices[(i + 1) % Vertices.size()]);
            if (side.PointOnSegment(point, Vertices[i], Vertices[(i + 1) % Vertices.size()])){
                return true;
            }
            if (line != side){
                if (line.parall(side)){
                    continue;
                }
                Point inters = line.intersection(side);
                if (inters != Vertices[i] && inters != Vertices[(i + 1) % Vertices.size()] && inters.x > point.x){
                    if (side.PointOnSegment(inters, Vertices[i], Vertices[(i + 1) % Vertices.size()])){
                        Inters_Numb++;
                    }
                }
                else if ((inters.x > point.x) && (inters.y < Vertices[i].y || inters.y < Vertices[(i + 1) % Vertices.size()].y)){
                    Inters_Numb++;
                }
            }
            else if (point.x > Vertices[i].x || point.x > Vertices[(i + 1) % Vertices.size()].x){
                Inters_Numb++;
            }
        }
        return Inters_Numb % 2 == 1;
    };

    void rotate(Point center, double angle) override{
        for (size_t i = 0; i < Vertices.size(); i++) {
            Vertices[i].rotate(center, angle);
        }
    };

    void reflex(Point center) override{
        for (size_t i = 0; i < Vertices.size(); i++) {
            Vertices[i].reflex(center);
        }
    };

    void reflex(Line axis) override{
        for (size_t i = 0; i < Vertices.size(); i++) {
            Vertices[i].reflex(axis.perp_base(Vertices[i]));
        }
    };

    void scale(Point center, double coefficient) override{
        for (size_t i = 0; i < Vertices.size(); i++) {
            Vertices[i].scale(center, coefficient);
        }
    };

    void print(){
        for (size_t i = 0; i < Vertices.size(); i++){
            std::cout << "(" << Vertices[i].x << ", " << Vertices[i].y << ") ";
        }
        std::cout << "\n";
    }
protected:
    std::vector<Point> Vertices;
};

class Ellipse: public Shape{
public:
    explicit Ellipse(Point p, Point q, double d){
        focuses_pair = std::make_pair(p, q);
        dist = d;
    }

    Ellipse() = default;

    std::pair<Point,Point> focuses() const{
        return focuses_pair;
    }

    std::pair<Line, Line> directrices() const{
        double c = focuses_pair.first.distance(center());
        double a = dist / 2;
        Line first = Line(Point(pow(a, 2)/c, 0), Point(pow(a, 2)/c, 1));
        Line second = Line(Point(-pow(a, 2)/c, 0), Point(-pow(a, 2)/c, 1));
        return std::make_pair(first, second);
    }

    double eccentricity() const{
        double c = focuses_pair.first.distance(center());
        double a = dist / 2;
        return  c / a;
    }

    Point center() const{
        return focuses_pair.first.center(focuses_pair.second);
    }

    double perimeter() const override {
        double a = dist / 2;
        double c = focuses_pair.first.distance(center());
        double b = pow(pow(a, 2) - pow(c, 2), 0.5);
        return PI * (3 * (a + b) - pow((3 * a + b) * (a + 3 * b), 0.5));
    }

    double area() const override {
        double a = dist / 2;
        double c = focuses_pair.first.distance(center());
        double b = pow(pow(a, 2) - pow(c, 2), 0.5);
        return PI * a * b;
    }

    bool operator==(const Shape& another) const override{
        const Shape* another_pointer = &another;
        auto ellipse = dynamic_cast<const Ellipse*>(another_pointer);
        if (ellipse){
            return focuses_pair == ellipse->focuses_pair || dist - ellipse->dist < epsilon;
        }
        return false;
    }

    bool operator!=(const Shape& another) const override{
        return !operator==(another);
    }

    bool isCongruentTo(const Shape& another) const override{
        const Shape* another_pointer = &another;
        auto ellipse = dynamic_cast<const Ellipse*>(another_pointer);
        if (ellipse){
            return fabs(focuses_pair.first.distance(focuses_pair.second) - ellipse->focuses_pair.first.distance(ellipse->focuses_pair.second)) < epsilon || fabs(dist - ellipse->dist) < epsilon;
        }
        return false;
    }

    bool isSimilarTo(const Shape& another) const override {
        const Shape* another_pointer = &another;
        auto ellipse = dynamic_cast<const Ellipse*>(another_pointer);
        if (ellipse){
            return fabs(ellipse->eccentricity() - eccentricity()) < epsilon;
        }
        return false;
    }

    bool containsPoint(Point point) const override {
        return (point.distance(focuses_pair.first) + point.distance(focuses_pair.second) < dist + epsilon);
    }

    void rotate(Point center, double angle) override {
        focuses_pair.first.rotate(center, angle);
        focuses_pair.second.rotate(center, angle);
    }

    void reflex(Point center) override {
        focuses_pair.first.reflex(center);
        focuses_pair.second.reflex(center);
    }

    void reflex(Line axis) override {
        focuses_pair.first.reflex(axis.perp_base(focuses_pair.first));
        focuses_pair.second.reflex(axis.perp_base(focuses_pair.second));
    }

    void scale(Point center, double coefficient) override {
        focuses_pair.first.scale(center, coefficient);
        focuses_pair.second.scale(center, coefficient);
        dist *= coefficient;
    }
protected:
    std::pair<Point, Point> focuses_pair;
    double dist;
};

class Circle: public Ellipse{
public:
    explicit Circle(Point center, double radius){
        focuses_pair = std::make_pair(center, center);
        dist = radius * 2;
    }

    double radius(){
        return dist / 2;
    }
};

class Rectangle: public Polygon{
public:
    explicit Rectangle(Point p, Point q, double quotient){
        if (quotient > 1){
            quotient = 1/quotient;
        }
        std::vector<Point> v(0);
        v.push_back(p);
        Point center = p.center(q);
        double angle = acos((1 - pow(quotient, 2))/(1 + pow(quotient, 2)));
        p.rotate(center, (angle * 180) / PI);
        v.push_back(p);
        v.push_back(q);
        q.rotate(center, (angle * 180) / PI);
        v.push_back(q);
        Vertices = v;
    }

    Rectangle() = default;

    Point center(){
        return Vertices[0].center(Vertices[2]);
    }

    std::pair<Line, Line> diagonals(){
        return std::make_pair(Line(Vertices[0], Vertices[2]), Line(Vertices[1], Vertices[3]));
    }
};

class Square: public Rectangle{
public:
    explicit Square(Point p, Point q){
        std::vector<Point> v(0);
        v.push_back(p);
        Point center = p.center(q);
        p.rotate(center, 90);
        v.push_back(p);
        v.push_back(q);
        q.rotate(center, 90);
        v.push_back(q);
        Vertices = v;
    }

    Square() = default;

    Circle circumscribedCircle() {
        return Circle(Vertices[0].center(Vertices[2]), Vertices[0].distance(Vertices[2]) / 2);
    }

    Circle inscribedCircle() {
        return Circle(Vertices[0].center(Vertices[2]), Vertices[0].distance(Vertices[1]) / 2);
    }
};

class Triangle: public Polygon{
public:
    Triangle(Point a, Point b, Point c){
        Vertices.push_back(a);
        Vertices.push_back(b);
        Vertices.push_back(c);
    };

    Triangle() = default;

    Circle circumscribedCircle(){
        Line first_side = Line(Vertices[0], Vertices[1]);
        Line first = first_side.perpendicular(Vertices[0].center(Vertices[1]));
        Line second_side = Line(Vertices[1], Vertices[2]);
        Line second = second_side.perpendicular(Vertices[1].center(Vertices[2]));
        Point center = first.intersection(second);
        return Circle(center, center.distance(Vertices[0]));
    }

    Circle inscribedCircle(){
        Point A = Vertices[0];
        A.scale(Vertices[1], Vertices[1].distance(Vertices[2]) / (Vertices[1].distance(Vertices[2]) + Vertices[2].distance(Vertices[0])));
        Line first = Line(Vertices[2], A);
        Point B = Vertices[1];
        B.scale(Vertices[2], Vertices[2].distance(Vertices[0]) / (Vertices[2].distance(Vertices[0]) + Vertices[0].distance(Vertices[1])));
        Line second = Line(Vertices[0], B);
        Point center = first.intersection(second);
        Line side = Line(Vertices[0], Vertices[2]);
        return Circle(center, center.distance(side.perp_base(center)));
    }

    Point centroid(){
        Line first = Line(Vertices[0], Vertices[1].center(Vertices[2]));
        Line second = Line(Vertices[1], Vertices[2].center(Vertices[0]));
        return first.intersection(second);
    }

    Point orthocenter(){
        Line first = Line(Vertices[0], Line(Vertices[1], Vertices[2]).perp_base(Vertices[0]));
        Line second = Line(Vertices[1], Line(Vertices[2], Vertices[0]).perp_base(Vertices[1]));
        return first.intersection(second);
    }

    Line EulerLine(){
        Point a = centroid();
        Point b = orthocenter();
        return Line(a, b);
    }

    Circle ninePointsCircle(){
        Point a = orthocenter();
        Point b = circumscribedCircle().center();
        Point center = a.center(b);
        return Circle(center, center.distance(Vertices[0].center(Vertices[1])));
    }
};

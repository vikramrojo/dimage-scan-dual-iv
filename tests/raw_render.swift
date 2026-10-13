// Compares how macOS shows a DNG (Apple's raw engine with default settings,
// as Preview and Photos do) and a TIFF (ColorSync, honouring its profile):
// both are rendered to linear sRGB and compared on an 8x8 grid of patches in
// CIE L*a*b*.  Prints: mean dE, max dE, mean |dL*|.
//   swiftc -O tests/raw_render.swift -o build/raw_render
//   build/raw_render scan.dng scan.tiff
import CoreImage
import Foundation

let lin = CGColorSpace(name: CGColorSpace.linearSRGB)!
let ctx = CIContext(options: [.workingColorSpace: lin, .outputColorSpace: lin])

func render(_ img: CIImage) -> ([Float], Int, Int) {
    let w = Int(img.extent.width), h = Int(img.extent.height)
    var b = [Float](repeating: 0, count: w * h * 4)
    ctx.render(img, toBitmap: &b, rowBytes: w * 16, bounds: img.extent, format: .RGBAf, colorSpace: lin)
    return (b, w, h)
}

func lab(_ r: Double, _ g: Double, _ b: Double) -> (Double, Double, Double) {
    let x = 0.4124564 * r + 0.3575761 * g + 0.1804375 * b
    let y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b
    let z = 0.0193339 * r + 0.1191920 * g + 0.9503041 * b
    func f(_ t: Double) -> Double { t > 216.0 / 24389 ? cbrt(t) : (24389.0 / 27 * t + 16) / 116 }
    let fx = f(x / 0.95047), fy = f(y), fz = f(z / 1.08883)
    return (116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz))
}

let a = CommandLine.arguments
guard a.count == 3, let raw = CIRAWFilter(imageURL: URL(fileURLWithPath: a[1])), let ri = raw.outputImage,
      let ti = CIImage(contentsOf: URL(fileURLWithPath: a[2])) else {
    print("usage: raw_render file.dng file.tiff (both must decode)"); exit(2)
}
let (r, w, h) = render(ri)
let (t, tw, th) = render(ti)
guard w == tw && h == th else { print("size mismatch \(w)x\(h) vs \(tw)x\(th)"); exit(2) }
var sum = 0.0, mx = 0.0, sl = 0.0, n = 0.0
let k = max(2, min(w, h) / 40)
for gy in 1..<8 { for gx in 1..<8 {
    var s1 = [0.0, 0.0, 0.0], s2 = [0.0, 0.0, 0.0]
    for y in (gy * h / 8 - k)..<(gy * h / 8 + k) { for x in (gx * w / 8 - k)..<(gx * w / 8 + k) {
        for c in 0..<3 { s1[c] += Double(r[(y * w + x) * 4 + c]); s2[c] += Double(t[(y * w + x) * 4 + c]) } } }
    let m = Double(4 * k * k)
    let l1 = lab(s1[0] / m, s1[1] / m, s1[2] / m), l2 = lab(s2[0] / m, s2[1] / m, s2[2] / m)
    let de = sqrt(pow(l1.0 - l2.0, 2) + pow(l1.1 - l2.1, 2) + pow(l1.2 - l2.2, 2))
    sum += de; mx = max(mx, de); sl += abs(l1.0 - l2.0); n += 1
} }
print(String(format: "%.2f %.2f %.2f", sum / n, mx, sl / n))

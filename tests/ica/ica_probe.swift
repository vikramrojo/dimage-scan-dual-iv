// Uses Apple's ImageCaptureCore (the framework behind Image Capture and
// Preview) to discover the dsdual4 AirScan bridge and run a scan through it.
// usage: ica_probe [seconds] [outdir]
import Foundation
import ImageCaptureCore

final class Probe: NSObject, ICDeviceBrowserDelegate, ICScannerDeviceDelegate {
    let browser = ICDeviceBrowser()
    var scanner: ICScannerDevice?
    let outDir: URL
    var done = false

    init(outDir: URL) {
        self.outDir = outDir
        super.init()
        browser.delegate = self
        let mask = ICDeviceTypeMask.scanner.rawValue | ICDeviceLocationTypeMask.local.rawValue
            | ICDeviceLocationTypeMask.bonjour.rawValue | ICDeviceLocationTypeMask.shared.rawValue
        browser.browsedDeviceTypeMask = ICDeviceTypeMask(rawValue: mask)!
        browser.start()
    }

    func deviceBrowser(_ browser: ICDeviceBrowser, didAdd device: ICDevice, moreComing: Bool) {
        print("found scanner: \(device.name ?? "?") transport=\(device.transportType ?? "?")")
        if let s = device as? ICScannerDevice, (device.name ?? "").contains("DiMAGE"), scanner == nil {
            scanner = s
            s.delegate = self
            s.requestOpenSession()
        }
    }
    func deviceBrowser(_ browser: ICDeviceBrowser, didRemove device: ICDevice, moreGoing: Bool) {}
    func didRemove(_ device: ICDevice) {}
    func device(_ device: ICDevice, didOpenSessionWithError error: Error?) {
        if let e = error { print("open session error: \(e)"); done = true; return }
        print("session open; functional units: \(scanner!.availableFunctionalUnitTypes)")
    }
    func device(_ device: ICDevice, didCloseSessionWithError error: Error?) {}
    func scannerDevice(_ scanner: ICScannerDevice, didSelect functionalUnit: ICScannerFunctionalUnit, error: Error?) {
        if let e = error { print("select unit error: \(e)"); done = true; return }
        let fu = functionalUnit
        print("unit: \(fu.type.rawValue) resolutions=\(fu.supportedResolutions.count) physical=\(fu.physicalSize)")
        fu.resolution = 300
        fu.scanArea = NSRect(origin: .zero, size: fu.physicalSize)
        fu.pixelDataType = .RGB
        fu.bitDepth = .depth8Bits
        scanner.transferMode = .fileBased
        scanner.downloadsDirectory = outDir
        scanner.documentName = "ica-scan"
        scanner.documentUTI = "public.jpeg"
        scanner.requestScan()
    }
    func scannerDevice(_ scanner: ICScannerDevice, didScanTo url: URL) { print("scanned to \(url.path)") }
    func scannerDevice(_ scanner: ICScannerDevice, didCompleteScanWithError error: Error?) {
        print("scan complete, error: \(String(describing: error))")
        done = true
    }
    func deviceDidBecomeReady(_ device: ICDevice) {
        if let s = device as? ICScannerDevice {
            print("ready")
            s.requestSelect(.flatbed)
        }
    }
}

let secs = CommandLine.arguments.count > 1 ? Double(CommandLine.arguments[1])! : 20
let out = URL(fileURLWithPath: CommandLine.arguments.count > 2 ? CommandLine.arguments[2] : NSTemporaryDirectory())
let p = Probe(outDir: out)
let end = Date().addingTimeInterval(secs)
while !p.done && Date() < end { RunLoop.main.run(until: Date().addingTimeInterval(0.2)) }
print(p.done ? "finished" : "timeout")

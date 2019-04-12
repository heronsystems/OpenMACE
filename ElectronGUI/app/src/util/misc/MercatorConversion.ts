export default class MercatorConversion {
    r_major: number // Equatorial Radius, WGS84
    r_minor: number // defined as constant
    f: number // 1/f=(a-b)/a , a=r_major, b=r_minor

    constructor() {
        this.r_major = 6378137.0 // Equatorial Radius, WGS84
        this.r_minor = 6356752.314245179 // defined as constant
        this.f = 298.257223563 // 1/f=(a-b)/a , a=r_major, b=r_minor
    }

    deg2rad = (deg: number): number => {
        let r = deg * (Math.PI / 180.0)
        return r
    }

    rad2deg = (rad: number): number => {
        let d = rad / (Math.PI / 180.0)
        return d
    }

    // lat lon to mercator
    ll2m = (lon: number, lat: number) => {
        // lat, lon in rad
        let x = this.r_major * this.deg2rad(lon)

        if (lat > 89.5) lat = 89.5
        if (lat < -89.5) lat = -89.5

        let temp = this.r_minor / this.r_major
        let es = 1.0 - temp * temp
        let eccent = Math.sqrt(es)

        let phi = this.deg2rad(lat)

        let sinphi = Math.sin(phi)

        let con = eccent * sinphi
        let com = 0.5 * eccent
        let con2 = Math.pow((1.0 - con) / (1.0 + con), com)
        let ts = Math.tan(0.5 * (Math.PI * 0.5 - phi)) / con2
        let y = 0 - this.r_major * Math.log(ts)
        let ret = { x: x, y: y }
        return ret
    }

    // mercator to lat lon
    m2ll = (x: number, y: number) => {
        let lat = this.rad2deg(x / this.r_major)

        let temp = this.r_minor / this.r_major
        let e = Math.sqrt(1.0 - temp * temp)
        let lon = this.rad2deg(this.pj_phi2(Math.exp(0 - y / this.r_major), e))

        let ret = { lat: lat, lon: lon }
        return ret
    }

    pj_phi2 = (ts: number, e: number) => {
        let N_ITER = 15
        let HALFPI = Math.PI / 2

        let TOL = 0.0000000001
        let eccnth: number
        let Phi: number
        let con: number
        let dphi: number
        let i: any
        eccnth = 0.5 * e
        Phi = HALFPI - 2 * Math.atan(ts)
        i = N_ITER
        do {
            con = e * Math.sin(Phi)
            dphi =
                HALFPI -
                2 * Math.atan(ts * Math.pow((1 - con) / (1 + con), eccnth)) -
                Phi
            Phi += dphi
        } while (Math.abs(dphi) > TOL && --i)
        return Phi
    }
}

// //usage
// let mercator = MercatorConversion.ll2m(47.6035525, 9.770602);//output mercator.x, mercator.y
// let latlon = MercatorConversion.m2ll(5299424.36041, 1085840.05328);//output latlon.lat, latlon.lon

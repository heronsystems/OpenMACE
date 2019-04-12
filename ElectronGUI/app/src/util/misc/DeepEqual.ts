// export default function deepEqual(obj1: Object, obj2: Object){
export default function deepEqual(obj1: any, obj2: any) {
    for (let key in obj1) {
        if (!obj2.hasOwnProperty(key)) {
            return false
        }
        // check null
        else if (obj1[key] === null && obj2[key] !== null) {
            return false
        }
        // if this is also an object recurse
        else if (typeof obj1[key] === "object") {
            if (!deepEqual(obj1[key], obj2[key])) {
                return false
            }
        } else if (typeof obj1[key] === "function") {
            throw new TypeError("deepEqual doesn't work on objects with fns")
        }
        // otherwise type is basic
        else if (obj1[key] !== obj2[key]) {
            return false
        }
    }
    return true
}

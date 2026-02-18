declare module './planner_wasm.js' {
  const factory: (options?: { locateFile?: (path: string) => string }) => Promise<{
    cwrap: (name: string, returnType: string | null, argTypes: string[]) => (...args: number[]) => number
    HEAPF32: Float32Array
  }>
  export default factory
}

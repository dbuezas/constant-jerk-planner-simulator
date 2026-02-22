import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { spawn } from 'node:child_process'
import path from 'node:path'

const wasmRoot = path.resolve(__dirname, 'src', 'wasm')
const wasmScript = path.join(wasmRoot, 'build-wasm.sh')
const wasmOut = path.join(wasmRoot, 'planner_wasm.js')

function buildWasm() {
  return new Promise<void>((resolve, reject) => {
    const child = spawn('bash', [wasmScript], { stdio: 'inherit' })
    child.on('exit', (code) => {
      if (code === 0) resolve()
      else reject(new Error(`WASM build failed with code ${code}`))
    })
  })
}

function wasmPlugin() {
  const watchedFiles = [
    wasmScript,
    path.join(wasmRoot, 'planner_wrapper.cpp'),
    path.join(wasmRoot, 'planner_wrapper.h'),
    path.resolve(__dirname, 'src', 'c', 'trajectory_constant_jerk.h'),
    path.resolve(__dirname, 'src', 'c', 'constant-jerk-planner.h'),
    path.resolve(__dirname, 'src', 'c', 'constant-jerk-planner.cpp'),
  ]

  return {
    name: 'planner-wasm-rebuild',
    async buildStart() {
      try {
        await buildWasm()
      } catch (err) {
        this.error((err as Error).message)
      }
    },
    configureServer(server: { watcher: { add: (paths: string[]) => void }; ws: { send: (payload: { type: string }) => void } }) {
      server.watcher.add(watchedFiles)
      server.watcher.on('change', async (file: string) => {
        if (!watchedFiles.includes(file)) return
        try {
          await buildWasm()
          server.ws.send({ type: 'full-reload' })
        } catch (err) {
          server.ws.send({ type: 'full-reload' })
          console.error(err)
        }
      })
    },
  }
}

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    react({
      babel: {
        plugins: [['babel-plugin-react-compiler']],
      },
    }),
    wasmPlugin(),
  ],
  server: {
    fs: {
      allow: [path.resolve(__dirname, '..')],
    },
  },
})

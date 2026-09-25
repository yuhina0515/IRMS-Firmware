// Builds manifest.json for a compiled firmware image and signs it with Ed25519.
// Usage: node scripts/make-manifest.mjs <bin> <version> <outDir> [notesFile]
// Env: FIRMWARE_SIGNING_KEY (PKCS#8 PEM), MIN_APP_VERSION (optional).
// The IRMS app verifies the exact bytes of manifest.json against manifest.json.sig with the
// public key compiled into IRMS_App_Tauri/src-tauri/src/firmware_update.rs.
import { createHash, createPrivateKey, createPublicKey, sign, verify } from 'node:crypto'
import { readFileSync, writeFileSync } from 'node:fs'
import { basename, join } from 'node:path'

const [bin, version, outDir, notesFile] = process.argv.slice(2)
if (!bin || !version || !outDir) throw new Error('usage: make-manifest.mjs <bin> <version> <outDir> [notesFile]')
const pem = process.env.FIRMWARE_SIGNING_KEY
if (!pem) throw new Error('FIRMWARE_SIGNING_KEY is not set')

const data = readFileSync(bin)
const manifest = {
  version,
  file: basename(bin),
  size: data.length,
  sha256: createHash('sha256').update(data).digest('hex'),
  md5: createHash('md5').update(data).digest('hex'),
  ...(process.env.MIN_APP_VERSION ? { minAppVersion: process.env.MIN_APP_VERSION } : {}),
  notes: notesFile ? readFileSync(notesFile, 'utf8').trim() : ''
}
const bytes = Buffer.from(JSON.stringify(manifest, null, 2) + '\n')
const key = createPrivateKey(pem)
const signature = sign(null, bytes, key)
// Self-check before publishing anything.
if (!verify(null, bytes, createPublicKey(key), signature)) throw new Error('self-verification failed')

writeFileSync(join(outDir, 'manifest.json'), bytes)
writeFileSync(join(outDir, 'manifest.json.sig'), signature.toString('base64') + '\n')
const raw = createPublicKey(key).export({ type: 'spki', format: 'der' }).subarray(-32).toString('base64')
console.log(`manifest for ${manifest.file} v${version} (${data.length} bytes), public key ${raw}`)

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';

//Set up scene and camera
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 0.1, 1000 );
//Set up renderer
const renderer = new THREE.WebGLRenderer();
renderer.setSize( window.innerWidth, window.innerHeight );
document.body.appendChild( renderer.domElement );
//Create "planet"
const sphereGeometry = new THREE.SphereGeometry( 1, 32, 32 );
const sphereMaterial = new THREE.MeshBasicMaterial( { color: 0x126567 } );
const sphere = new THREE.Mesh( sphereGeometry, sphereMaterial );
//Create object on planet
const boxGeometry = new THREE.BoxGeometry( 0.5, 0.5, 0.5 );
const boxMaterial = new THREE.MeshBasicMaterial( { color: 0x8B4513 } );
const box = new THREE.Mesh( boxGeometry, boxMaterial );
box.position.y = 1;
//Create independant object
const ballGeometry = new THREE.SphereGeometry( 0.2, 32, 32 );
const ballMaterial = new THREE.MeshBasicMaterial( { color: 0x0000ff } );
const ball = new THREE.Mesh( ballGeometry, ballMaterial );
ball.position.y = 3;

//group box and sphere
const group = new THREE.Group();
group.add(sphere);
group.add(box);

//add meshes to scene
scene.add(group);
scene.add(ball);
//Set camera
camera.position.z = 5;

//Variables for rapier physics
let world;
let ballRB;
let sphereRB;

//Variables for custom gravity
const ATTRACTION_STRENGTH = 1;
const ATTRACTION_EPS = 1e-3;
const ATTRACTION_MAX_FORCE = 1;

// initialize Rapier and physics objects
async function initPhysics() {
  await RAPIER.init(); // loads WASM

  //Create world without gravity
  world = new RAPIER.World({ x: 0, y: 0, z: 0 });

  //Create RB and collider for sphere
  const sphereBodyDesc = RAPIER.RigidBodyDesc.fixed()
    .setTranslation(sphere.position.x, sphere.position.y, sphere.position.z);
  sphereRB = world.createRigidBody(sphereBodyDesc);
  const sphereCollider = RAPIER.ColliderDesc.ball(1.0)
    .setTranslation(0, 0, 0);
  world.createCollider(sphereCollider, sphereRB);

  //Create collider for box
  const boxCollider = RAPIER.ColliderDesc.cuboid(0.25, 0.25, 0.25)
    .setTranslation(box.position.x, box.position.y, box.position.z); // local offset;
  world.createCollider(boxCollider, sphereRB);

  //Create RB and collider for ball
  const ballBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(ball.position.x, ball.position.y, ball.position.z);
  ballRB = world.createRigidBody(ballBodyDesc);
  const ballCollider = RAPIER.ColliderDesc.ball(0.2)
    .setTranslation(0, 0, 0);
  world.createCollider(ballCollider, ballRB);

}

function animate() {
  // step the physics world before rendering
  if (world) {
    world.step();

    // continuous attraction: apply a force on the ball toward the sphere

    if (ballRB && sphereRB) {
      const bPos = ballRB.translation();
      const sPos = sphereRB.translation();
      const dx = sPos.x - bPos.x;
      const dy = sPos.y - bPos.y;
      const dz = sPos.z - bPos.z;
      const distSq = dx*dx + dy*dy + dz*dz + ATTRACTION_EPS;
      const invDist = 1 / Math.sqrt(distSq);
      // normalized direction
      const nx = dx * invDist;
      const ny = dy * invDist;
      const nz = dz * invDist;
      // inverse-square style magnitude
      let mag = ATTRACTION_STRENGTH / distSq;
      if (mag > ATTRACTION_MAX_FORCE) mag = ATTRACTION_MAX_FORCE;
      const force = { x: nx * mag, y: ny * mag, z: nz * mag };
      
      // applyImpulse-only: scale force by timestep to produce an impulse
      const dt = 1 / 60; // approximate timestep (adjust if needed)
      const impulse = { x: force.x * dt, y: force.y * dt, z: force.z * dt };
      // assume applyImpulse exists on the Rapier rigid body
      ballRB.applyImpulse(impulse, true);
    }

    if (ballRB) {
      const t = ballRB.translation();
      const rot = ballRB.rotation();
      if (Number.isFinite(t.x) && Number.isFinite(t.y) && Number.isFinite(t.z)) {
        ball.position.set(t.x, t.y, t.z);
      }
      if (Number.isFinite(rot.x) && Number.isFinite(rot.y) && Number.isFinite(rot.z) && Number.isFinite(rot.w)) {
        ball.quaternion.set(rot.x, rot.y, rot.z, rot.w);
      }
    }

    // sync group from sphereRB
    if (sphereRB) {
      const t = sphereRB.translation();
      const rot = sphereRB.rotation();
      if (Number.isFinite(t.x) && Number.isFinite(t.y) && Number.isFinite(t.z)) {
        group.position.set(t.x, t.y, t.z);
      }
      if (Number.isFinite(rot.x) && Number.isFinite(rot.y) && Number.isFinite(rot.z) && Number.isFinite(rot.w)) {
        group.quaternion.set(rot.x, rot.y, rot.z, rot.w);
      }
    }

  }

  renderer.render( scene, camera );

}

// call init
initPhysics().then(() => {
  renderer.setAnimationLoop( animate );
}).catch(err => {
  console.error('Rapier initialization failed:', err);
})

let moving = false;

renderer.domElement.addEventListener('mousedown', () => {
  moving = true;
});

renderer.domElement.addEventListener('mouseup', () => {
  moving = false;
});

renderer.domElement.addEventListener('mousemove', (event) => {
  if (moving) {
    const deltaX = event.movementX || event.mozMovementX || event.webkitMovementX || 0;
    const deltaY = event.movementY || event.mozMovementY || event.webkitMovementY || 0;

    //rotate sphere
    if (sphereRB != null) {
      const rb = world.getRigidBody(sphereRB);
      if (rb) {
        //store quaternion
        const rot = rb.rotation();
        const currentQ = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);

        //Build small incremental rotations based on mouse movement
        const sensitivity = 0.01;
        const angY = deltaX * sensitivity;
        const angX = deltaY * sensitivity;
        const deltaQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(angX, angY, 0, 'XYZ'));

        //apply incremental rotation
        const newQ = deltaQ.multiply(currentQ);

        rb.setRotation({ x: newQ.x, y: newQ.y, z: newQ.z, w: newQ.w }, true);
      }
    }
  }
});
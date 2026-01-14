/**
 * Setup script for the ROS2 educational module
 * This script helps initialize the Docusaurus environment
 */

const fs = require('fs');
const path = require('path');

console.log('Setting up ROS2 educational module environment...');

// Check if website directory exists
const websiteDir = path.join(__dirname, '..', 'website');
if (!fs.existsSync(websiteDir)) {
  console.error('Error: website directory not found!');
  process.exit(1);
}

// Check if required directories exist
const requiredDirs = [
  path.join(websiteDir, 'src', 'components'),
  path.join(websiteDir, 'static', 'img'),
  path.join(__dirname, '..', 'docs', 'modules', 'ros2-humanoid-control')
];

for (const dir of requiredDirs) {
  if (!fs.existsSync(dir)) {
    console.log(`Creating directory: ${dir}`);
    fs.mkdirSync(dir, { recursive: true });
  }
}

console.log('✅ ROS2 educational module environment setup complete!');
console.log('\nTo start the development server:');
console.log('  cd website');
console.log('  npm start');
/**
 * Content validation script for the ROS2 educational module
 * This script validates the educational content for quality and completeness
 */

const fs = require('fs');
const path = require('path');

console.log('Validating ROS2 educational module content...');

// Define expected content files
const expectedFiles = [
  '../docs/modules/ros2-humanoid-control/index.md',
  '../docs/modules/ros2-humanoid-control/chapter-1-ros2-fundamentals.md',
  '../docs/modules/ros2-humanoid-control/chapter-2-python-agents-rclpy.md',
  '../docs/modules/ros2-humanoid-control/chapter-3-urdf-modeling.md'
];

let allFilesExist = true;

for (const file of expectedFiles) {
  const filePath = path.join(__dirname, file);
  if (fs.existsSync(filePath)) {
    const stats = fs.statSync(filePath);
    const content = fs.readFileSync(filePath, 'utf8');

    console.log(`✅ ${file} - Size: ${stats.size} bytes`);

    // Basic validation: check if file has reasonable content length
    if (content.length < 50) {
      console.warn(`⚠️  ${file} - File seems to have very little content`);
    }
  } else {
    console.error(`❌ ${file} - File does not exist`);
    allFilesExist = false;
  }
}

if (allFilesExist) {
  console.log('\n✅ All expected content files exist!');
  console.log('Content validation passed.');
} else {
  console.log('\n❌ Some content files are missing!');
  console.log('Please create the missing files before proceeding.');
  process.exit(1);
}
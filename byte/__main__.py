#!/usr/bin/env python3
"""
RobotDog Command Line Interface
==============================
Entry point for running RobotDog as a command-line application.
"""

import sys
import argparse
from . import RobotDog

def main():
    """Main entry point for the RobotDog CLI."""
    parser = argparse.ArgumentParser(
        description="RobotDog - Quadruped Robot Control System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  byte --help                    # Show this help message
  byte --version                 # Show version information
  byte --demo                    # Run a demo sequence
        """
    )
    
    parser.add_argument(
        '--version',
        action='version',
        version='RobotDog %s' % RobotDog.__version__ if hasattr(RobotDog, '__version__') else 'unknown'
    )
    
    parser.add_argument(
        '--demo',
        action='store_true',
        help='Run a demo sequence'
    )
    
    args = parser.parse_args()
    
    if args.demo:
        print("Starting RobotDog demo...")
        try:
            dog = RobotDog()
            print("RobotDog initialized successfully!")
            print("Demo completed.")
            dog.close()
        except Exception as e:
            print(f"Error during demo: {e}")
            sys.exit(1)
    else:
        parser.print_help()

if __name__ == '__main__':
    main() 
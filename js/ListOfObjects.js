export function ListOfObjects() {
    return [
      {
        name: 'Water Tank',
        type: 'box',
        center: [-10, -20, 42],
        extents: [10, 5, 5],
        rotation: [0, 0, 0]
    
       },
       {
         name: 'Main Deck',
         type: 'box',
         center: [-2.9, -13, 35],
         extents: [50, 10.1, 32],
         rotation: [0, 0, 0]
       },
       {
        name: 'Lower Deck',
        type: 'box',
        center: [3, 8, 32],
        extents: [38, 1, 10],
        rotation: [0, 0, 0]
      },
      {
         name: 'Radar Dome',
         type: 'cylinder',
         center: [40, 40, 10],
         extents: [5,2], // For a cylinder: [radius, height]
         rotation: [0, 0, 0]
       },
       {
        name: 'Fuel Tank',
        type: 'cylinder',
        center: [20, 9, 38],
        extents: [3, 9], // For a cylinder: [radius, height]
        rotation: [0, 0, 0]
      },


    ];
  }
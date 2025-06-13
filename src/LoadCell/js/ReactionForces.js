fetch('equations.json')
  .then(response => response.json())
  .then(data => {
      // data now contains your equations as strings
      // Use math.js or another library to parse/evaluate them
      console.log(data);
 
 
 
 
 

 
    })
  .catch(error => console.error('Error fetching JSON:', error));
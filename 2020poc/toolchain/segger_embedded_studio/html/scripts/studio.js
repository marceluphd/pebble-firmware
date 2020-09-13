
StudioSolution = function(name)
{
  this.solutionDocument_ = document.implementation.createDocument(null, "solution", document.implementation.createDocumentType("CrossStudio_Project_File"));
  var solutionElement = this.solutionDocument_.documentElement;
  solutionElement.setAttribute("Name", name);
  solutionElement.setAttribute("target", studio.getStudioTargetNumber());
  solutionElement.setAttribute("version", "2");
}

StudioSolution.solutionDocument_;

StudioSolution.prototype.rootElement = function()
{
  return this.solutionDocument_.documentElement;
}

StudioSolution.prototype.addConfiguration = function(parentElement, name)
{
  var configurationElement = this.solutionDocument_.createElement("configuration");
  parentElement.appendChild(configurationElement);
  configurationElement.setAttribute("Name", name);
  return configurationElement;
}

StudioSolution.prototype.addProject = function(parentElement, name)
{
  var projectElement = this.solutionDocument_.createElement("project");
  parentElement.appendChild(projectElement);
  projectElement.setAttribute("Name", name);
  return projectElement;
}

StudioSolution.prototype.addImport = function(parentElement, filename)
{
  var importElement = this.solutionDocument_.createElement("import");
  parentElement.appendChild(importElement);
  importElement.setAttribute("file_name", filename);
  return importElement;
}

StudioSolution.prototype.addFolder = function(parentElement, name)
{
  var folderElement = this.solutionDocument_.createElement("folder");
  parentElement.appendChild(folderElement);
  folderElement.setAttribute("Name", name);
  return folderElement;
}

StudioSolution.prototype.addFile = function(parentElement, name)
{
  var fileElement = this.solutionDocument_.createElement("file");
  parentElement.appendChild(fileElement);
  fileElement.setAttribute("file_name", name);
  return fileElement;
}

StudioSolution.prototype.writeToFile = function(path, indent)
{
  studio.writeXMLToFile(path, this.solutionDocument_, indent);
}

studio.writeXMLToFile = function(path, doc, indent)
{
  var serializer = new XMLSerializer();
  var xmlString = serializer.serializeToString(doc);
  xmlString = studio.formatXMLString(xmlString, indent);
  studio.writeStringToFile(path, xmlString);
}

studio.setBusy = function(busy)
{
  if (busy)
    {
      if ($('.studio-busy-animation').length == 0)
        $('body').append('<div class="studio-busy-animation" />');
    }
  else
    $('.studio-busy-animation').remove();
}

window.onerror = function(message, url, line, column, error)
{
  studio.setBusy(false);
  if (url !== 'undefined')
    {
      message += "</p><p><b>File:</b> " + url;
      if (line)
        message += "<br/><b>Line:</b> " + line;
      if (column)
        message += "<br/><b>Column:</b> " + column;
      close = true;
    }

  studio.showErrorMessageBox("Script Error", "<p>" + message + "</p>");
  return true;
}


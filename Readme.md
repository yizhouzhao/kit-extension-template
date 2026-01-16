# Kit Extension Template

A Kit Extension Template for Omniverse Kit/Isaac Sim.

As the official [Isaac Sim Extension Generator](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/utilities/extension_template_generator.html) is too complicated, I am creating this repo for simplicity.

Extension window Screenshot:

![create](img/create_gear.png)

## Download the [release](https://github.com/yizhouzhao/GearSimulator/releases/tag/0.2) or clone the this repository 

```
git clone https://github.com/yizhouzhao/kit-extension-template
```

Upzip or locate the root folder as <your-path-to-GearSimulator>

The file structure of this extension should look like:

```
<your-path-to-this-extension>
    └───exts    [Omniverse extenstion]
         └───your.app.template
                └───config  [Extension config]
                └───icons   [Extension icons]
                └───your/... [source code]
    └───img   
    │   .gitignore 
    │   README.md  
    ......
```

## Add extension to Omniverse

1. **[Open extension manager]** After opening Omniverse Code, go to `Menu` -> `Window` -> `Extension`

![add_extension](img/extension_manager.png)


2. **[Add this extension to Omniverse]** Click the  <img src="https://github.githubassets.com/images/icons/emoji/unicode/2699.png?v8" width="18"> button, add absolute extension path to `Extension Search Paths`.

![add_extension](img/extension_path.png)

Finally, you can search the `your.app.template` (locate in the `Third Party tag`) and enable this extension.

> **Note**:
> The extension path to add is: `<your-path-to-this-repo>/exts`

![add_extension](img/add_extension.PNG)


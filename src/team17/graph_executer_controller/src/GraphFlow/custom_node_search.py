#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Custom Node Search Dialog
This module provides a more robust replacement for the NodeGraphQt search dialog
that may not work properly on some systems.
"""
from Qt import QtWidgets, QtCore, QtGui

class CustomNodeSearchWidget(QtWidgets.QDialog):
    """
    Custom dialog for searching nodes in the graph.
    """
    
    def __init__(self, parent=None, node_dict=None):
        super(CustomNodeSearchWidget, self).__init__(parent)
        self.setWindowTitle('节点搜索')
        self.setWindowFlags(QtCore.Qt.Tool)
        self.setFixedSize(400, 300)
        
        # Store node dictionary
        self._node_dict = node_dict or {}
        self._node_names = list(sorted(self._node_dict.keys()))
        
        # Create the search input field
        self._search_line_edit = QtWidgets.QLineEdit()
        self._search_line_edit.setPlaceholderText('搜索节点...')
        self._search_line_edit.textChanged.connect(self._on_search_changed)
        
        # Create the node list
        self._node_list_widget = QtWidgets.QListWidget()
        self._node_list_widget.itemDoubleClicked.connect(self._on_item_double_clicked)
        
        # Reset button
        self._reset_button = QtWidgets.QPushButton('清除')
        self._reset_button.clicked.connect(self._on_reset_clicked)
        
        # Button layout
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addWidget(self._reset_button)
        button_layout.addStretch()
        
        # Add button
        self._add_button = QtWidgets.QPushButton('添加节点')
        self._add_button.clicked.connect(self._on_item_double_clicked)
        button_layout.addWidget(self._add_button)
        
        # Main layout
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self._search_line_edit)
        layout.addWidget(self._node_list_widget)
        layout.addLayout(button_layout)
        
        # Populate the list initially
        self._populate_node_list()
        
        # Set focus to search field
        self._search_line_edit.setFocus()
        
    def _populate_node_list(self, filter_text=None):
        """
        Populate the node list with node names, optionally filtered by text.
        """
        self._node_list_widget.clear()
        
        for name in self._node_names:
            if filter_text and filter_text.lower() not in name.lower():
                continue
            
            item = QtWidgets.QListWidgetItem(name)
            item.setData(QtCore.Qt.UserRole, name)
            self._node_list_widget.addItem(item)
            
        # Select the first item if available
        if self._node_list_widget.count() > 0:
            self._node_list_widget.setCurrentRow(0)
    
    def _on_search_changed(self, text):
        """
        Handle search text changes.
        """
        self._populate_node_list(text)
    
    def _on_reset_clicked(self):
        """
        Reset the search field.
        """
        self._search_line_edit.clear()
        self._populate_node_list()
    
    def _on_item_double_clicked(self, item=None):
        """
        Handle item double click or add button click.
        """
        if not item:
            item = self._node_list_widget.currentItem()
            
        if not item:
            return
        
        node_name = item.data(QtCore.Qt.UserRole)
        node_class = self._node_dict.get(node_name)
        
        if node_class:
            # Create the node in the graph
            graph = self.parent()
            if hasattr(graph, 'create_node'):
                pos = graph.get_view().mapToScene(
                    graph.get_view().viewport().rect().center()
                )
                node = graph.create_node(node_class, pos=pos)
                
                # Close the dialog
                self.accept()
    
    def keyPressEvent(self, event):
        """
        Override keypress event to handle Enter key.
        """
        if event.key() == QtCore.Qt.Key_Return or event.key() == QtCore.Qt.Key_Enter:
            self._on_item_double_clicked()
            return
            
        super(CustomNodeSearchWidget, self).keyPressEvent(event)
    
    @staticmethod
    def create_and_show(graph, position=None):
        """
        Create and show the search dialog.
        
        Args:
            graph: The NodeGraph instance
            position: Optional position for the dialog
        """
        # Get the node factory
        node_dict = {}
        
        # Extract node types from graph
        if hasattr(graph, '_node_factory'):
            for name, node_class in graph._node_factory.items():
                # Skip Backdrop nodes
                if name == 'Backdrop':
                    continue
                node_dict[name] = node_class
        
        # Create dialog
        dialog = CustomNodeSearchWidget(graph, node_dict)
        
        # Set position if provided
        if position:
            dialog.move(position)
        
        # Show the dialog
        dialog.show()
        dialog.raise_()  # Bring to front
        dialog.activateWindow()  # Activate the window
        
        return dialog 